/* USB MJPEG / CSI ISP NV12 -> Cedar hardware H.264 -> RTSP.
 * Capture timestamps use CLOCK_MONOTONIC; wall-clock corrections cannot affect RTP.
 * The CPU decodes JPEG/converts pixels. All H.264 encoding is on the A733 VE2.
 */
#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE 700
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libswscale/swscale.h>
#include <vencoder.h>
#include <memoryAdapter.h>
#include <AWIspApi.h>
#include "camera_output.h"

/* ISP602 exposes four Q8.8 Bayer WB gains through this public control API. */
extern int isp_get_attr_cfg(int isp_id, uint32_t control, int32_t *value);
extern int isp_set_attr_cfg(int isp_id, uint32_t control, int32_t *value);
extern int isp_get_cfg(int isp_id, uint8_t group, uint32_t controls, void *value);
extern int isp_set_cfg(int isp_id, uint8_t group, uint32_t controls, void *value);
extern int isp_update(int isp_id);
#define ISP_CTRL_WB_MGAIN 7

static volatile sig_atomic_t stopping;
static int64_t io_deadline_us;
static int64_t mono_us(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000000 + now.tv_nsec / 1000;
}
static void stop_signal(int signal_number) { (void)signal_number; stopping = 1; }
static int interrupted(void *unused) {
    (void)unused;
    return stopping || (io_deadline_us && mono_us() > io_deadline_us);
}
static int xioctl(int fd, unsigned long request, void *argument) {
    int result;
    do { result = ioctl(fd, request, argument); } while (result < 0 && errno == EINTR && !stopping);
    return result;
}
static int failure(const char *operation, int result) {
    fprintf(stderr, "hwstream: %s failed (%d): %s\n", operation, result, strerror(errno));
    return -1;
}
#define TRY(call) do { int rc_ = (call); if (rc_ < 0) { failure(#call, rc_); goto cleanup; } } while (0)
#define VTRY(call) do { int rc_ = (call); if (rc_ != 0) { failure(#call, rc_); goto cleanup; } } while (0)

struct CaptureBuffer { void *data; size_t size; };
struct Capture {
    int fd;
    unsigned count;
    struct CaptureBuffer buffers[8];
    bool streaming;
    enum v4l2_buf_type type;
    bool nv12;
    unsigned width, height;
    struct v4l2_plane planes[8][VIDEO_MAX_PLANES];
    AWIspApi *isp;
    int isp_id;
    bool isp_initialized, isp_running;
    uint16_t wb_gains[4];
    bool wb_manual;
    bool wb_auto;
    bool ccm_enabled;
    bool input_full_range;
    int color_temperature;
    struct CameraOutput output;
    int control_fd, sensor_fd, exposure_auto, manual_exposure, manual_gain;
    char control_path[108];
    int64_t last_wb_update;
    int lens_fd, lens_position, af_stage, af_candidate, af_limit, af_step, af_best_position;
    unsigned af_settle, af_samples, af_bad_frames, af_tick;
    double af_sum, af_best_metric, af_metric, af_locked_metric;
    int64_t af_next_scan;
};

static int lens_transfer(struct Capture *camera, unsigned char *data, unsigned size, bool read) {
    struct i2c_msg message = {.addr = 0x0c, .flags = read ? I2C_M_RD : 0, .len = size, .buf = data};
    struct i2c_rdwr_ioctl_data transaction = {.msgs = &message, .nmsgs = 1};
    return ioctl(camera->lens_fd, I2C_RDWR, &transaction) == 1 ? 0 : -1;
}

static int lens_read(struct Capture *camera, unsigned char reg, unsigned char *value) {
    if (lens_transfer(camera, &reg, 1, false)) return -1;
    return lens_transfer(camera, value, 1, true);
}

static int lens_move(struct Capture *camera, int target) {
    if (target < 450) target = 450;
    if (target > 950) target = 950;
    while (camera->lens_position != target) {
        unsigned char status = 0;
        if (lens_read(camera, 0x05, &status)) return -1;
        for (unsigned tries = 0; status && tries < 10; ++tries) {
            struct timespec delay = {.tv_nsec = 1000000};
            nanosleep(&delay, NULL);
            if (lens_read(camera, 0x05, &status)) return -1;
        }
        if (status) return -1;
        int next = camera->lens_position + (target > camera->lens_position ? 16 : -16);
        if ((target > camera->lens_position && next > target) ||
            (target < camera->lens_position && next < target)) next = target;
        unsigned char command[3] = {0x03, (next >> 8) & 3, next & 255};
        if (lens_transfer(camera, command, 3, false)) return -1;
        camera->lens_position = next;
        struct timespec delay = {.tv_nsec = 1000000};
        nanosleep(&delay, NULL);
    }
    return 0;
}

static void lens_init(struct Capture *camera) {
    camera->lens_fd = -1;
    if (!camera->nv12) return;
    const char *autofocus = getenv("BOT_CAM_AUTOFOCUS");
    if (autofocus && !strcmp(autofocus, "0")) {
        fprintf(stderr, "hwstream: fixed-focus M12 module selected; autofocus disabled\n");
        return;
    }
    camera->lens_fd = open("/dev/i2c-3", O_RDWR | O_CLOEXEC);
    if (camera->lens_fd < 0) return;
    unsigned char wake[2] = {0x02, 0x00}, high = 0, low = 0;
    if (lens_transfer(camera, wake, 2, false) || lens_read(camera, 0x03, &high) || lens_read(camera, 0x04, &low)) {
        close(camera->lens_fd);
        camera->lens_fd = -1;
        fprintf(stderr, "hwstream: autofocus actuator did not respond\n");
        return;
    }
    camera->lens_position = ((high & 3) << 8) | low;
    camera->af_stage = 0;
    camera->af_next_scan = mono_us() + 5000000;
    fprintf(stderr, "hwstream: DW9817 lens detected at I2C3/0x0c; contrast autofocus enabled\n");
}

static double focus_metric(const struct Capture *camera, const unsigned char *pixels) {
    double score = 0;
    unsigned count = 0;
    for (unsigned y = camera->height / 4; y < camera->height * 3 / 4; y += 4) {
        for (unsigned x = camera->width / 4; x < camera->width * 3 / 4; x += 4) {
            size_t index = (size_t)y * camera->width + x;
            if (pixels[index] < 24 || pixels[index] > 235) continue;
            int dx = pixels[index + 2] - pixels[index - 2];
            int dy = pixels[index + 2 * camera->width] - pixels[index - 2 * camera->width];
            score += dx * dx + dy * dy;
            ++count;
        }
    }
    return count ? score / count : 0;
}

static void autofocus_update(struct Capture *camera, const unsigned char *pixels, int64_t now) {
    if (camera->lens_fd < 0) return;
    if (camera->af_stage == 0 && now < camera->af_next_scan) return;
    if (camera->af_stage == 3 && (++camera->af_tick % 8)) return;
    if ((camera->af_stage == 1 || camera->af_stage == 2) && camera->af_settle) {
        --camera->af_settle;
        return;
    }
    camera->af_metric = focus_metric(camera, pixels);
    if (camera->af_stage == 3) {
        if (camera->af_locked_metric > 5 && camera->af_metric < camera->af_locked_metric * 0.50)
            ++camera->af_bad_frames;
        else camera->af_bad_frames = 0;
        if (camera->af_bad_frames < 14 || now < camera->af_next_scan) return;
        camera->af_stage = 0;
    }
    if (camera->af_stage == 0) {
        if (now < camera->af_next_scan) return;
        camera->af_stage = 1;
        camera->af_candidate = 450;
        camera->af_limit = 950;
        camera->af_step = 70;
        camera->af_best_metric = -1;
        if (lens_move(camera, camera->af_candidate)) goto lens_error;
        camera->af_settle = 8;
        camera->af_samples = 0;
        camera->af_sum = 0;
        return;
    }
    camera->af_sum += camera->af_metric;
    if (++camera->af_samples < 4) return;
    double metric = camera->af_sum / camera->af_samples;
    if (metric > camera->af_best_metric) {
        camera->af_best_metric = metric;
        camera->af_best_position = camera->af_candidate;
    }
    camera->af_candidate += camera->af_step;
    if (camera->af_candidate > camera->af_limit) {
        if (camera->af_stage == 1) {
            int center = camera->af_best_position;
            camera->af_stage = 2;
            camera->af_candidate = center - 50 < 450 ? 450 : center - 50;
            camera->af_limit = center + 50 > 950 ? 950 : center + 50;
            camera->af_step = 10;
            camera->af_best_metric = -1;
        } else {
            if (lens_move(camera, camera->af_best_position)) goto lens_error;
            camera->af_stage = 3;
            camera->af_locked_metric = camera->af_best_metric;
            camera->af_bad_frames = 0;
            camera->af_next_scan = now + 15000000;
            fprintf(stderr, "hwstream: autofocus locked at lens=%d, sharpness=%.2f (contrast method)\n",
                    camera->af_best_position, camera->af_best_metric);
            return;
        }
    }
    if (lens_move(camera, camera->af_candidate)) goto lens_error;
    camera->af_settle = 8;
    camera->af_samples = 0;
    camera->af_sum = 0;
    return;
lens_error:
    fprintf(stderr, "hwstream: lens control failed; preserving current lens position\n");
    close(camera->lens_fd);
    camera->lens_fd = -1;
}

static double clamp_double(double value, double low, double high) {
    return value < low ? low : value > high ? high : value;
}

#include "camera_control.h"

/* Correct the generic ISP preset using a small unsaturated scene sample.
 * Only ISP Bayer gains are changed; the full-rate NV12 image remains in hardware.
 */
static void correct_white_balance(struct Capture *camera, const unsigned char *pixels,
                                  int64_t now) {
    if (!camera->nv12 || now - camera->last_wb_update < 1000000) return;
    camera->last_wb_update = now;
    if (!camera->wb_manual) {
        uint16_t current[4] = {0};
        if (isp_get_attr_cfg(camera->isp_id, ISP_CTRL_WB_MGAIN, (int32_t *)current) != 0)
            return;
        for (int i = 0; i < 4; ++i) {
            if (current[i] < 64 || current[i] > 4095) return;
            camera->wb_gains[i] = current[i];
        }
        struct v4l2_control control = {.id = V4L2_CID_AUTO_WHITE_BALANCE, .value = 0};
        if (xioctl(camera->fd, VIDIOC_S_CTRL, &control) < 0) return;
        if (isp_set_attr_cfg(camera->isp_id, ISP_CTRL_WB_MGAIN, (int32_t *)camera->wb_gains) != 0)
            return;
        camera->wb_manual = true;
    }
    if (!camera->wb_auto) return;
    if (camera_actual_gain(camera) >= 128) return; /* Freeze colour estimates in high-gain darkness. */
    size_t y_size = (size_t)camera->width * camera->height;
    double red = 0, green = 0, blue = 0;
    unsigned samples = 0;
    double neutral_red = 0, neutral_green = 0, neutral_blue = 0;
    unsigned neutral_samples = 0;
    unsigned step_x = (camera->width / 64 + 1) & ~1U;
    unsigned step_y = (camera->height / 48 + 1) & ~1U;
    if (!step_x) step_x = 2;
    if (!step_y) step_y = 2;
    for (unsigned y = camera->height / 8; y < camera->height * 7 / 8; y += step_y) {
        for (unsigned x = camera->width / 8; x < camera->width * 7 / 8; x += step_x) {
            double yy = camera->input_full_range ? pixels[(size_t)y * camera->width + x]
                : 1.164 * (pixels[(size_t)y * camera->width + x] - 16);
            size_t offset = y_size + (size_t)(y / 2) * camera->width + (x & ~1U);
            double u = pixels[offset] - 128, v = pixels[offset + 1] - 128;
            double r = yy + (camera->input_full_range ? 1.402 : 1.596) * v;
            double g = yy - (camera->input_full_range ? .344136 : .392) * u - (camera->input_full_range ? .714136 : .813) * v;
            double b = yy + (camera->input_full_range ? 1.772 : 2.017) * u;
            if (r < 25 || g < 25 || b < 25 || r > 235 || g > 235 || b > 235) continue;
            red += r; green += g; blue += b; ++samples;
            double high = r > g ? r : g;
            if (b > high) high = b;
            double low = r < g ? r : g;
            if (b < low) low = b;
            /* Prefer bright, low-chroma walls/neutral references. Do not let
             * brown furniture or coloured clothing set the white point.
             */
            if (yy > 110 && low > high * 0.72) {
                neutral_red += r; neutral_green += g; neutral_blue += b;
                ++neutral_samples;
            }
        }
    }
    if (neutral_samples >= 100) {
        red = neutral_red; green = neutral_green; blue = neutral_blue;
        samples = neutral_samples;
    } else return;  /* No neutral reference: do not chase furniture colours or dark noise. */
    if (samples < 100 || red <= 0 || blue <= 0) return;
    double r_ratio = clamp_double(green / red, 0.75, 1.33);
    double b_ratio = clamp_double(green / blue, 0.75, 1.33);
    uint16_t proposal[4]; memcpy(proposal,camera->wb_gains,sizeof(proposal));
    proposal[0] = (uint16_t)clamp_double(proposal[0] * (0.75 + 0.25 * r_ratio),64,4095);
    proposal[3] = (uint16_t)clamp_double(proposal[3] * (0.75 + 0.25 * b_ratio),64,4095);
    int temperature=imx708_temperature_from_gains(proposal);
    if (temperature>camera->color_temperature+125) temperature=camera->color_temperature+125;
    if (temperature<camera->color_temperature-125) temperature=camera->color_temperature-125;
    if (imx708_set_temperature(camera,temperature)) return;
    fprintf(stderr, "hwstream: ISP white balance R/Gr/Gb/B=%u/%u/%u/%u; scene RGB=%.1f/%.1f/%.1f\n",
            camera->wb_gains[0], camera->wb_gains[1], camera->wb_gains[2], camera->wb_gains[3],
            red / samples, green / samples, blue / samples);
}

static int open_camera(struct Capture *camera, const char *device, unsigned width,
                       unsigned height, unsigned fps) {
    camera_output_default(&camera->output);
    camera->wb_auto = true;
    camera->manual_exposure = 8000;
    camera->manual_gain = 16;
    camera->fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (camera->fd < 0) return failure("open camera", -1);
    struct v4l2_capability capability = {0};
    if (xioctl(camera->fd, VIDIOC_QUERYCAP, &capability) < 0) return failure("QUERYCAP", -1);
    unsigned caps = capability.capabilities & V4L2_CAP_DEVICE_CAPS ?
        capability.device_caps : capability.capabilities;
    camera->nv12 = (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
    camera->type = camera->nv12 ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    camera->width = width;
    camera->height = height;
    if (camera->nv12) {
        int input = 0;
        if (xioctl(camera->fd, VIDIOC_S_INPUT, &input) < 0) return failure("S_INPUT CSI", -1);
    }
    struct v4l2_format format = {0};
    format.type = camera->type;
    if (camera->nv12) {
        format.fmt.pix_mp.width = width;
        format.fmt.pix_mp.height = height;
        format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        format.fmt.pix_mp.field = V4L2_FIELD_NONE;
        format.fmt.pix_mp.num_planes = 1;
        /* A733 extension: explicitly request limited-range BT.601 for H.264. */
        format.fmt.pix_mp.colorspace = 34;
        format.fmt.pix_mp.quantization = V4L2_QUANTIZATION_LIM_RANGE;
        format.fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_601;
    } else {
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        format.fmt.pix.field = V4L2_FIELD_ANY;
    }
    if (xioctl(camera->fd, VIDIOC_S_FMT, &format) < 0) return failure("S_FMT", -1);
    unsigned accepted_w = camera->nv12 ? format.fmt.pix_mp.width : format.fmt.pix.width;
    unsigned accepted_h = camera->nv12 ? format.fmt.pix_mp.height : format.fmt.pix.height;
    unsigned accepted_fmt = camera->nv12 ? format.fmt.pix_mp.pixelformat : format.fmt.pix.pixelformat;
    if (accepted_w != width || accepted_h != height ||
        accepted_fmt != (camera->nv12 ? V4L2_PIX_FMT_NV12 : V4L2_PIX_FMT_MJPEG) ||
        (camera->nv12 && format.fmt.pix_mp.num_planes != 1)) {
        fprintf(stderr, "hwstream: camera did not accept requested format/dimensions\n");
        return -1;
    }
    struct v4l2_streamparm rate = {0};
    rate.type = camera->type;
    rate.parm.capture.timeperframe.numerator = 1;
    rate.parm.capture.timeperframe.denominator = fps;
    if (xioctl(camera->fd, VIDIOC_S_PARM, &rate) < 0) return failure("S_PARM", -1);
    if (camera->nv12) {
        camera->isp = CreateAWIspApi();
        if (!camera->isp || camera->isp->ispApiInit() != 0) return failure("AW ISP init", -1);
        camera->isp_initialized = true;
        char resolved[4096];
        if (!realpath(device, resolved)) return failure("resolve CSI video device", -1);
        const char *number = strrchr(resolved, '/');
        if (!number || strncmp(number + 1, "video", 5)) return failure("CSI video node name", -1);
        camera->isp_id = camera->isp->ispGetIspId(atoi(number + 6));
        if (camera->isp_id < 0 || camera->isp->ispStart(camera->isp_id) != 0)
            return failure("AW ISP start", -1);
        camera->isp_running = true;
        /* ISP602's TEST_ENABLE block is 33 packed byte flags. The fallback
         * OV13850 lens mesh, colour matrix and colour-enhancement LUT are not
         * calibrated for IMX708. Preserve all other modules, including AE,
         * demosaic, black-level correction, gamma and WB.
         * Layout verified against this SDK's exported setters/getter.
         */
        uint8_t modules[256] = {0};
        uint8_t black_level_config[4096] = {0};
        int black_level_bytes = isp_get_cfg(camera->isp_id, 0x05, 0x40, black_level_config);
        fprintf(stderr, "hwstream: ISP black-level configuration bytes=%d, prefix=", black_level_bytes);
        for (int i = 0; i < black_level_bytes && i < 64; ++i)
            fprintf(stderr, "%02x", black_level_config[i]);
        fprintf(stderr, "\n");
        /* ISP602 packs a 2-byte trigger and fourteen four-channel signed
         * black offsets. Keep the visually verified -64 setting: -256
         * clips scene information on this pipeline despite the 12-bit FE.
         */
        if (black_level_bytes == 114) {
            int16_t optical_black = -64;
            for (int i = 2; i < black_level_bytes; i += 2)
                memcpy(black_level_config + i, &optical_black, sizeof(optical_black));
            if (isp_set_cfg(camera->isp_id, 0x05, 0x40, black_level_config) == 114)
                fprintf(stderr, "hwstream: IMX708 optical-black offsets applied: -64\n");
        }
        int module_bytes = isp_get_cfg(camera->isp_id, 0x01, 0x20, modules);
        fprintf(stderr, "hwstream: ISP602 module block size=%d, LSC/MSC/CCM/CEM=%u/%u/%u/%u\n",
                module_bytes, modules[17], modules[18], modules[23], modules[29]);
        if (module_bytes == 33) {
            modules[17] = modules[18] = modules[23] = modules[29] = 0;
            /* The OV13850 HDR tone curves lift IMX708 shadows and flatten
             * chroma. Use the normal gamma path for this non-HDR mode.
             */
            modules[19] = modules[26] = modules[27] = 0;
            int applied = isp_set_cfg(camera->isp_id, 0x01, 0x20, modules);
            if (applied == 33 && isp_update(camera->isp_id) == 0)
                fprintf(stderr, "hwstream: disabled foreign OV13850 shading / colour transforms / HDR tone mapping for IMX708\n");
        }
        camera->isp->ispSetFpsRanage(camera->isp_id, fps);
    }
    struct v4l2_requestbuffers request = {0};
    request.count = 4;
    request.type = camera->type;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(camera->fd, VIDIOC_REQBUFS, &request) < 0) return failure("REQBUFS", -1);
    if (request.count < 2 || request.count > 8) return failure("unsupported capture buffer count", -1);
    camera->count = request.count;
    for (unsigned i = 0; i < camera->count; ++i) {
        struct v4l2_buffer buffer = {0};
        buffer.type = request.type;
        buffer.memory = request.memory;
        buffer.index = i;
        if (camera->nv12) {
            buffer.m.planes = camera->planes[i];
            buffer.length = VIDEO_MAX_PLANES;
        }
        if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buffer) < 0) return failure("QUERYBUF", -1);
        size_t length = camera->nv12 ? buffer.m.planes[0].length : buffer.length;
        off_t offset = camera->nv12 ? buffer.m.planes[0].m.mem_offset : buffer.m.offset;
        camera->buffers[i].size = length;
        camera->buffers[i].data = mmap(NULL, length, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, camera->fd, offset);
        if (camera->buffers[i].data == MAP_FAILED) return failure("mmap", -1);
        if (xioctl(camera->fd, VIDIOC_QBUF, &buffer) < 0) return failure("QBUF", -1);
    }
    enum v4l2_buf_type type = camera->type;
    if (xioctl(camera->fd, VIDIOC_STREAMON, &type) < 0) return failure("STREAMON", -1);
    camera->streaming = true;
    if (camera->nv12) {
        /* Apply image controls through the owning descriptor; a second VIN
         * capture open cannot safely control the camera while it streams.
         */
        struct v4l2_control controls[] = {
            {.id = V4L2_CID_CONTRAST, .value = 0},
            {.id = V4L2_CID_SATURATION, .value = 0},
        };
        for (unsigned i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
            if (xioctl(camera->fd, VIDIOC_S_CTRL, &controls[i]) < 0)
                fprintf(stderr, "hwstream: ISP image control 0x%x unavailable\n", controls[i].id);
        fprintf(stderr, "hwstream: IMX708 neutral SDR image calibration\n");
        fprintf(stderr, "hwstream: M12 output correction: measured contrast, highlight shoulder, neutral chroma\n");
    }
    lens_init(camera);
    if (camera->nv12 && imx708_install_profile(camera)) return failure("IMX708 ISP colour profile",-1);
    if (camera_control_open(camera) < 0) return failure("camera control socket", -1);
    fprintf(stderr, "hwstream: %s %s %ux%u requested %u fps, %u capture buffers, latest-frame selection\n",
            device, camera->nv12 ? "CSI ISP NV12" : "MJPEG", width, height, fps, camera->count);
    return 0;
}

static void close_camera(struct Capture *camera) {
    camera_control_close(camera);
    if (camera->fd < 0) return;
    if (camera->streaming) {
        enum v4l2_buf_type type = camera->type;
        ioctl(camera->fd, VIDIOC_STREAMOFF, &type);
    }
    if (camera->isp_running) {
        camera->isp->ispStop(camera->isp_id);
        camera->isp->ispWaitToExit(camera->isp_id);
    }
    if (camera->isp_initialized) camera->isp->ispApiUnInit();
    if (camera->isp) DestroyAWIspApi(camera->isp);
    if (camera->lens_fd >= 0) close(camera->lens_fd);
    for (unsigned i = 0; i < camera->count; ++i)
        if (camera->buffers[i].data && camera->buffers[i].data != MAP_FAILED)
            munmap(camera->buffers[i].data, camera->buffers[i].size);
    close(camera->fd);
}

/* Drain completed buffers and return only the newest one, never a growing FIFO. */
static int latest_frame(struct Capture *camera, struct v4l2_buffer *latest, uint64_t *dropped) {
    struct pollfd ready = {.fd = camera->fd, .events = POLLIN};
    int result = poll(&ready, 1, 1500);
    if (result < 0 && errno == EINTR) return 0;
    if (result <= 0 || (ready.revents & (POLLERR | POLLHUP | POLLNVAL)))
        return failure("camera stopped delivering frames", -1);
    bool have_frame = false;
    for (unsigned i = 0; i < camera->count; ++i) {
        struct v4l2_buffer buffer = {0};
        struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
        buffer.type = camera->type;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (camera->nv12) {
            buffer.m.planes = planes;
            buffer.length = VIDEO_MAX_PLANES;
        }
        if (xioctl(camera->fd, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) break;
            return failure("DQBUF", -1);
        }
        if (camera->nv12) buffer.bytesused = planes[0].bytesused;
        if (buffer.index >= camera->count || buffer.bytesused > camera->buffers[buffer.index].size)
            return failure("invalid camera buffer", -1);
        if (camera->nv12) {
            memcpy(camera->planes[buffer.index], planes, sizeof(planes));
            buffer.m.planes = camera->planes[buffer.index];
        }
        if (have_frame) {
            if (xioctl(camera->fd, VIDIOC_QBUF, latest) < 0) return failure("QBUF old frame", -1);
            ++*dropped;
        }
        *latest = buffer;
        have_frame = true;
    }
    return have_frame;
}

static void snapshot(const char *path, const unsigned char *data, size_t size) {
    char temporary[4096];
    if (snprintf(temporary, sizeof(temporary), "%s.tmp", path) >= (int)sizeof(temporary)) return;
    FILE *file = fopen(temporary, "wb");
    if (!file) return;
    bool written = fwrite(data, 1, size, file) == size;
    if (fclose(file) != 0) written = false;
    if (written) rename(temporary, path);
    else unlink(temporary);
}

static void progress(uint64_t frames, uint64_t dropped, uint64_t timestamp_fallbacks, int64_t pts,
                     int64_t started, double age_ms, double encode_ms, const struct Capture *camera) {
    double elapsed = (mono_us() - started) / 1000000.0;
    int64_t seconds = pts / 1000000;
    printf("frame=%" PRIu64 "\nfps=%.2f\ndup_frames=0\ndrop_frames=%" PRIu64
           "\nout_time=%02" PRId64 ":%02" PRId64 ":%02" PRId64 ".%06" PRId64
           "\nspeed=%.3fx\ncapture_to_publish_ms=%.2f\nhardware_encode_ms=%.2f"
           "\ntimestamp_fallbacks=%" PRIu64 "\nautofocus_position=%d\nautofocus_sharpness=%.2f"
           "\nautofocus_state=%s\nprogress=continue\n",
           frames, frames / elapsed, dropped, seconds / 3600, (seconds / 60) % 60,
           seconds % 60, pts % 1000000, (pts / 1000000.0) / elapsed, age_ms, encode_ms, timestamp_fallbacks,
           camera->lens_position, camera->af_metric,
           camera->lens_fd < 0 ? (getenv("BOT_CAM_AUTOFOCUS") && !strcmp(getenv("BOT_CAM_AUTOFOCUS"), "0") ? "fixed" : "unavailable")
           : camera->af_stage == 3 ? "locked" : camera->af_stage ? "scanning" : "warming");
    fflush(stdout);
}

int main(int argc, char **argv) {
    if (argc != 8) {
        fprintf(stderr, "usage: %s DEVICE WIDTH HEIGHT FPS BITRATE RTSP_URL SNAPSHOT_PATH\n", argv[0]);
        return 2;
    }
    const char *device = argv[1], *url = argv[6], *snapshot_path = argv[7];
    int width = atoi(argv[2]), height = atoi(argv[3]), fps = atoi(argv[4]), bitrate = atoi(argv[5]);
    if (width < 16 || width > 4096 || height < 16 || height > 4096 ||
        (width & 1) || (height & 1) || fps < 1 || fps > 120 || bitrate < 100000 || bitrate > 50000000)
        return 2;
    signal(SIGTERM, stop_signal);
    signal(SIGINT, stop_signal);
    signal(SIGPIPE, SIG_IGN);
    av_log_set_level(AV_LOG_WARNING);
    avformat_network_init();

    int exit_code = 1;
    struct Capture camera = {.fd = -1, .lens_fd = -1, .control_fd = -1, .sensor_fd = -1};
    struct ScMemOpsS *memory = NULL;
    VideoEncoder *encoder = NULL;
    bool memory_open = false, encoder_initialized = false, buffers_allocated = false, mux_open = false;
    AVCodecContext *decoder = NULL;
    AVFrame *decoded = NULL;
    AVPacket *jpeg = NULL, *encoded = NULL;
    AVCodecContext *jpeg_encoder = NULL;
    AVFrame *jpeg_frame = NULL;
    AVPacket *snapshot_packet = NULL;
    struct SwsContext *snapshot_converter = NULL;
    struct SwsContext *converter = NULL;
    unsigned char *corrected_pixels = NULL;
    enum AVPixelFormat conversion_format = AV_PIX_FMT_NONE;
    AVFormatContext *mux = NULL;
    AVStream *track = NULL;
    VencHeaderData header = {0};

    memory = MemAdapterGetOpsS();
    if (!memory) goto cleanup;
    VTRY(CdcMemOpen(memory));
    memory_open = true;
    encoder = VideoEncCreate(VENC_CODEC_H264);
    if (!encoder) goto cleanup;
    VencH264Param settings = {0};
    settings.sProfileLevel.nProfile = VENC_H264ProfileBaseline;
    settings.sProfileLevel.nLevel = (width > 1920 || fps > 30) ? VENC_H264Level51 : VENC_H264Level4;
    settings.sQPRange.nMinqp = settings.sQPRange.nMinPqp = 15;
    settings.sQPRange.nMaxqp = settings.sQPRange.nMaxPqp = 42;
    settings.sQPRange.nQpInit = 28;
    settings.nFramerate = settings.nSrcFramerate = fps;
    settings.nBitrate = bitrate;
    settings.nMaxKeyInterval = fps / 2 ? fps / 2 : 1;
    settings.nCodingMode = VENC_FRAME_CODING;
    settings.sRcParam.eRcMode = AW_CBR;
    VTRY(VideoEncSetParameter(encoder, VENC_IndexParamH264Param, &settings));
    VencH264VideoSignal colors = {0};
    colors.video_format = DEFAULT;
    colors.full_range_flag = 0;
    colors.transfer_characteristics = 6;
    colors.matrix_coefficients = 6;
    colors.src_colour_primaries = colors.dst_colour_primaries = VENC_BT601;
    VTRY(VideoEncSetParameter(encoder, VENC_IndexParamH264VideoSignal, &colors));
    int stride = (width + 15) & ~15;
    int aligned_height = (height + 15) & ~15;
    VencBaseConfig base = {0};
    base.nInputWidth = base.nDstWidth = width;
    base.nInputHeight = base.nDstHeight = height;
    base.nStride = stride;
    base.eInputFormat = VENC_PIXEL_YUV420SP;
    base.memops = memory;
    VTRY(VideoEncInit(encoder, &base));
    encoder_initialized = true;
    VencAllocateBufferParam allocation = {
        .nBufferNum = 1, .nSizeY = stride * aligned_height, .nSizeC = stride * aligned_height / 2};
    VTRY(AllocInputBuffer(encoder, &allocation));
    buffers_allocated = true;
    VTRY(VideoEncGetParameter(encoder, VENC_IndexParamH264SPSPPS, &header));

    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!codec) goto cleanup;
    decoder = avcodec_alloc_context3(codec);
    decoded = av_frame_alloc();
    jpeg = av_packet_alloc();
    encoded = av_packet_alloc();
    if (!decoder || !decoded || !jpeg || !encoded) goto cleanup;
    decoder->thread_count = 1;
    decoder->flags |= AV_CODEC_FLAG_LOW_DELAY;
    TRY(avcodec_open2(decoder, codec, NULL));

    TRY(avformat_alloc_output_context2(&mux, NULL, "rtsp", url));
    mux->interrupt_callback.callback = interrupted;
    mux->flags |= AVFMT_FLAG_FLUSH_PACKETS;
    mux->max_delay = 0;
    track = avformat_new_stream(mux, NULL);
    if (!track) goto cleanup;
    track->time_base = (AVRational){1, 90000};
    track->avg_frame_rate = (AVRational){fps, 1};
    AVCodecParameters *parameters = track->codecpar;
    parameters->codec_type = AVMEDIA_TYPE_VIDEO;
    parameters->codec_id = AV_CODEC_ID_H264;
    parameters->width = width;
    parameters->height = height;
    parameters->format = AV_PIX_FMT_YUV420P;
    parameters->profile = FF_PROFILE_H264_BASELINE;
    parameters->level = (width > 1920 || fps > 30) ? 51 : 40;
    parameters->bit_rate = bitrate;
    parameters->color_range = AVCOL_RANGE_MPEG;
    parameters->color_space = AVCOL_SPC_SMPTE170M;
    parameters->color_primaries = AVCOL_PRI_BT470BG;
    parameters->color_trc = AVCOL_TRC_SMPTE170M;
    parameters->extradata = av_mallocz(header.nLength + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!parameters->extradata) goto cleanup;
    memcpy(parameters->extradata, header.pBuffer, header.nLength);
    parameters->extradata_size = header.nLength;
    AVDictionary *options = NULL;
    av_dict_set(&options, "rtsp_transport", "tcp", 0);
    av_dict_set(&options, "muxdelay", "0", 0);
    io_deadline_us = mono_us() + 5000000;
    int mux_result = avformat_write_header(mux, &options);
    av_dict_free(&options);
    TRY(mux_result);
    mux_open = true;
    io_deadline_us = 0;
    TRY(open_camera(&camera, device, width, height, fps));
    if (camera.nv12) {
        corrected_pixels = av_malloc((size_t)width * height * 3 / 2);
        if (!corrected_pixels) goto cleanup;
        const AVCodec *snapshot_codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
        if (!snapshot_codec) goto cleanup;
        jpeg_encoder = avcodec_alloc_context3(snapshot_codec);
        jpeg_frame = av_frame_alloc();
        snapshot_packet = av_packet_alloc();
        if (!jpeg_encoder || !jpeg_frame || !snapshot_packet) goto cleanup;
        jpeg_encoder->width = width;
        jpeg_encoder->height = height;
        jpeg_encoder->pix_fmt = AV_PIX_FMT_YUVJ420P;
        jpeg_encoder->time_base = (AVRational){1, 1};
        jpeg_encoder->thread_count = 1;
        jpeg_encoder->color_range = AVCOL_RANGE_JPEG;
        jpeg_encoder->colorspace = AVCOL_SPC_SMPTE170M;
        TRY(avcodec_open2(jpeg_encoder, snapshot_codec, NULL));
        jpeg_frame->format = AV_PIX_FMT_YUVJ420P;
        jpeg_frame->width = width;
        jpeg_frame->height = height;
        TRY(av_frame_get_buffer(jpeg_frame, 32));
        snapshot_converter = sws_getContext(width, height, AV_PIX_FMT_NV12,
            width, height, AV_PIX_FMT_YUVJ420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
        if (!snapshot_converter) goto cleanup;
        const int *snapshot_matrix = sws_getCoefficients(SWS_CS_ITU601);
        TRY(sws_setColorspaceDetails(snapshot_converter, snapshot_matrix, 0, snapshot_matrix, 1,
                                     0, 1 << 16, 1 << 16));
    }
    fprintf(stderr, "hwstream: Cedar VE2 hardware H.264 Baseline active, %d bps; monotonic RTP timestamps\n", bitrate);

    int64_t started = mono_us(), last_snapshot = 0, last_report = 0, last_pts = -1;
    uint64_t frames = 0, dropped = 0, timestamp_fallbacks = 0;
    while (!stopping) {
        camera_control_poll(&camera);
        /* A stuck driver/encoder must be restarted by the console supervisor. */
        alarm(5);
        struct v4l2_buffer capture = {0};
        int ready = latest_frame(&camera, &capture, &dropped);
        TRY(ready);
        if (!ready) continue;
        int64_t now = mono_us();
        int64_t capture_us = (int64_t)capture.timestamp.tv_sec * 1000000 + capture.timestamp.tv_usec;
        if ((capture.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC ||
            capture_us > now || now - capture_us > 5000000) {
            capture_us = now;
            ++timestamp_fallbacks;
        }
        int64_t pts = capture_us - started;
        if (pts < 0 || pts <= last_pts || now - capture_us > 200000 ||
            !capture.bytesused || (capture.flags & V4L2_BUF_FLAG_ERROR)) {
            ++dropped;
            TRY(xioctl(camera.fd, VIDIOC_QBUF, &capture));
            continue;
        }
        if (!camera.nv12) {
            av_packet_unref(jpeg);
            TRY(av_new_packet(jpeg, capture.bytesused));
            memcpy(jpeg->data, camera.buffers[capture.index].data, capture.bytesused);
            TRY(xioctl(camera.fd, VIDIOC_QBUF, &capture));
        }
        if (!camera.nv12 && (avcodec_send_packet(decoder, jpeg) < 0 || avcodec_receive_frame(decoder, decoded) < 0)) {
            avcodec_flush_buffers(decoder);
            ++dropped;
            continue;
        }
        if (!camera.nv12 && (decoded->width != width || decoded->height != height)) {
            fprintf(stderr, "hwstream: JPEG dimensions changed unexpectedly\n");
            goto cleanup;
        }
        /* Cache by the original JPEG pixel format ourselves. sws_getCachedContext
         * compares against its internally normalized YUVJ format and otherwise
         * recreates the converter on every frame on this FFmpeg version.
         */
        if (!camera.nv12 && (!converter || conversion_format != decoded->format)) {
            sws_freeContext(converter);
            converter = sws_getContext(width, height, decoded->format,
                width, height, AV_PIX_FMT_NV12, SWS_FAST_BILINEAR, NULL, NULL, NULL);
            if (!converter) goto cleanup;
            conversion_format = decoded->format;
            /* UVC JPEG is full-range BT.601; preserve the matrix and convert range. */
            const int *matrix = sws_getCoefficients(SWS_CS_ITU601);
            TRY(sws_setColorspaceDetails(converter, matrix, 1, matrix, 0, 0, 1 << 16, 1 << 16));
        }
        VencInputBuffer input = {0};
        VTRY(GetOneAllocInputBuffer(encoder, &input));
        if (!frames) {
            memset(input.pAddrVirY, 16, allocation.nSizeY);
            memset(input.pAddrVirC, 128, allocation.nSizeC);
        }
        input.nPts = pts;
        input.nWidth = width;
        input.nHeight = height;
        unsigned char *planes[4] = {input.pAddrVirY, input.pAddrVirC, NULL, NULL};
        int linesizes[4] = {stride, stride, 0, 0};
        if (camera.nv12) {
            size_t y_size = (size_t)width * height;
            if (capture.bytesused < y_size + y_size / 2) {
                fprintf(stderr, "hwstream: incomplete CSI NV12 buffer\n");
                goto cleanup;
            }
            const unsigned char *pixels = camera.buffers[capture.index].data;
            correct_white_balance(&camera, pixels, now);
            /* DMA mappings are not normal cached RAM: copy in a burst before
             * SIMD processing, and never read back from the encoder mapping.
             */
            memcpy(corrected_pixels, pixels, y_size + y_size / 2);
            TRY(xioctl(camera.fd, VIDIOC_QBUF, &capture));
            autofocus_update(&camera, corrected_pixels, now);
            if (camera.input_full_range) {
                camera_full_to_limited(corrected_pixels,width,width,height,false);
                camera_full_to_limited(corrected_pixels+y_size,width,width,height/2,true);
            }
            if (camera.output.contrast!=100 || camera.output.saturation!=100 ||
                camera.output.black_point!=0 || camera.output.gamma!=100) {
                camera_output_correct(&camera.output, corrected_pixels, width, width, height, false, corrected_pixels, width);
                camera_output_correct(&camera.output, corrected_pixels + y_size, width, width, height / 2, true,
                                      corrected_pixels + y_size, width);
            }
            if (stride == width) {
                memcpy(input.pAddrVirY, corrected_pixels, y_size);
                memcpy(input.pAddrVirC, corrected_pixels + y_size, y_size / 2);
            } else {
                for (int y = 0; y < height; ++y)
                    memcpy(input.pAddrVirY + (size_t)y * stride, corrected_pixels + (size_t)y * width, width);
                for (int y = 0; y < height / 2; ++y)
                    memcpy(input.pAddrVirC + (size_t)y * stride, corrected_pixels + y_size + (size_t)y * width, width);
            }
        } else {
            TRY(sws_scale(converter, (const uint8_t * const *)decoded->data, decoded->linesize,
                          0, height, planes, linesizes));
        }
        if (camera.nv12 && now - last_snapshot >= 1000000) {
            TRY(av_frame_make_writable(jpeg_frame));
            const unsigned char *snapshot_planes[4] = {input.pAddrVirY, input.pAddrVirC, NULL, NULL};
            TRY(sws_scale(snapshot_converter, snapshot_planes, linesizes, 0, height,
                          jpeg_frame->data, jpeg_frame->linesize));
            jpeg_frame->pts = frames;
            TRY(avcodec_send_frame(jpeg_encoder, jpeg_frame));
            av_packet_unref(snapshot_packet);
            TRY(avcodec_receive_packet(jpeg_encoder, snapshot_packet));
            snapshot(snapshot_path, snapshot_packet->data, snapshot_packet->size);
            last_snapshot = now;
        }
        VTRY(FlushCacheAllocInputBuffer(encoder, &input));
        VTRY(AddOneInputBuffer(encoder, &input));
        int64_t encode_start = mono_us();
        VTRY(VideoEncodeOneFrame(encoder));
        double encode_ms = (mono_us() - encode_start) / 1000.0;
        VTRY(AlreadyUsedInputBuffer(encoder, &input));
        VTRY(ReturnOneAllocInputBuffer(encoder, &input));
        VencOutputBuffer output = {0};
        VTRY(GetOneBitstreamFrame(encoder, &output));
        bool keyframe = output.nFlag & VENC_BUFFERFLAG_KEYFRAME;
        size_t prefix = keyframe ? header.nLength : 0;
        av_packet_unref(encoded);
        TRY(av_new_packet(encoded, prefix + output.nSize0 + output.nSize1 + output.nSize2));
        unsigned char *destination = encoded->data;
        if (prefix) { memcpy(destination, header.pBuffer, prefix); destination += prefix; }
        memcpy(destination, output.pData0, output.nSize0); destination += output.nSize0;
        if (output.nSize1) { memcpy(destination, output.pData1, output.nSize1); destination += output.nSize1; }
        if (output.nSize2) memcpy(destination, output.pData2, output.nSize2);
        encoded->pts = encoded->dts = av_rescale_q(output.nPts, (AVRational){1, 1000000}, track->time_base);
        encoded->duration = 0; /* Actual capture spacing drives RTP; never invent missing frames. */
        encoded->stream_index = track->index;
        encoded->flags = keyframe ? AV_PKT_FLAG_KEY : 0;
        VTRY(FreeOneBitStreamFrame(encoder, &output));
        io_deadline_us = mono_us() + 500000;
        TRY(av_write_frame(mux, encoded));
        io_deadline_us = 0;
        last_pts = pts;
        ++frames;
        now = mono_us();
        double age_ms = (now - capture_us) / 1000.0;
        if (!camera.nv12 && now - last_snapshot >= 1000000) {
            snapshot(snapshot_path, jpeg->data, jpeg->size);
            last_snapshot = now;
        }
        if (now - last_report >= 1000000) {
            progress(frames, dropped, timestamp_fallbacks, pts, started, age_ms, encode_ms, &camera);
            last_report = now;
        }
        av_frame_unref(decoded);
    }
    exit_code = 0;

cleanup:
    alarm(0);
    close_camera(&camera);
    if (mux_open) {
        io_deadline_us = mono_us() + 200000;
        av_write_trailer(mux);
    }
    if (mux) avformat_free_context(mux);
    sws_freeContext(converter);
    av_free(corrected_pixels);
    sws_freeContext(snapshot_converter);
    av_packet_free(&snapshot_packet);
    av_frame_free(&jpeg_frame);
    avcodec_free_context(&jpeg_encoder);
    av_packet_free(&jpeg);
    av_packet_free(&encoded);
    av_frame_free(&decoded);
    avcodec_free_context(&decoder);
    if (buffers_allocated) ReleaseAllocInputBuffer(encoder);
    if (encoder_initialized) VideoEncUnInit(encoder);
    if (encoder) VideoEncDestroy(encoder);
    if (memory_open) CdcMemClose(memory);
    avformat_network_deinit();
    return stopping ? 0 : exit_code;
}
