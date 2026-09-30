/* USB MJPEG -> Cedar hardware H.264 -> RTSP. One bounded, latest-frame pipeline.
 * Capture timestamps use CLOCK_MONOTONIC; wall-clock corrections cannot affect RTP.
 * The CPU decodes JPEG/converts pixels. All H.264 encoding is on the A733 VE2.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
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

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libswscale/swscale.h>
#include <vencoder.h>
#include <memoryAdapter.h>

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
};

static int open_camera(struct Capture *camera, const char *device, unsigned width,
                       unsigned height, unsigned fps) {
    camera->fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (camera->fd < 0) return failure("open camera", -1);
    struct v4l2_capability capability = {0};
    if (xioctl(camera->fd, VIDIOC_QUERYCAP, &capability) < 0) return failure("QUERYCAP", -1);
    struct v4l2_format format = {0};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(camera->fd, VIDIOC_S_FMT, &format) < 0) return failure("S_FMT MJPEG", -1);
    if (format.fmt.pix.width != width || format.fmt.pix.height != height ||
        format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
        fprintf(stderr, "hwstream: camera did not accept requested MJPEG dimensions\n");
        return -1;
    }
    struct v4l2_streamparm rate = {0};
    rate.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rate.parm.capture.timeperframe.numerator = 1;
    rate.parm.capture.timeperframe.denominator = fps;
    if (xioctl(camera->fd, VIDIOC_S_PARM, &rate) < 0) return failure("S_PARM", -1);
    struct v4l2_requestbuffers request = {0};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(camera->fd, VIDIOC_REQBUFS, &request) < 0) return failure("REQBUFS", -1);
    if (request.count < 2 || request.count > 8) return failure("unsupported capture buffer count", -1);
    camera->count = request.count;
    for (unsigned i = 0; i < camera->count; ++i) {
        struct v4l2_buffer buffer = {0};
        buffer.type = request.type;
        buffer.memory = request.memory;
        buffer.index = i;
        if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buffer) < 0) return failure("QUERYBUF", -1);
        camera->buffers[i].size = buffer.length;
        camera->buffers[i].data = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, camera->fd, buffer.m.offset);
        if (camera->buffers[i].data == MAP_FAILED) return failure("mmap", -1);
        if (xioctl(camera->fd, VIDIOC_QBUF, &buffer) < 0) return failure("QBUF", -1);
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(camera->fd, VIDIOC_STREAMON, &type) < 0) return failure("STREAMON", -1);
    camera->streaming = true;
    fprintf(stderr, "hwstream: %s %ux%u requested %u fps, %u capture buffers, latest-frame selection\n",
            device, width, height, fps, camera->count);
    return 0;
}

static void close_camera(struct Capture *camera) {
    if (camera->fd < 0) return;
    if (camera->streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(camera->fd, VIDIOC_STREAMOFF, &type);
    }
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
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (xioctl(camera->fd, VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) break;
            return failure("DQBUF", -1);
        }
        if (buffer.index >= camera->count || buffer.bytesused > camera->buffers[buffer.index].size)
            return failure("invalid camera buffer", -1);
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
                     int64_t started, double age_ms, double encode_ms) {
    double elapsed = (mono_us() - started) / 1000000.0;
    int64_t seconds = pts / 1000000;
    printf("frame=%" PRIu64 "\nfps=%.2f\ndup_frames=0\ndrop_frames=%" PRIu64
           "\nout_time=%02" PRId64 ":%02" PRId64 ":%02" PRId64 ".%06" PRId64
           "\nspeed=%.3fx\ncapture_to_publish_ms=%.2f\nhardware_encode_ms=%.2f"
           "\ntimestamp_fallbacks=%" PRIu64 "\nprogress=continue\n",
           frames, frames / elapsed, dropped, seconds / 3600, (seconds / 60) % 60,
           seconds % 60, pts % 1000000, (pts / 1000000.0) / elapsed, age_ms, encode_ms, timestamp_fallbacks);
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
        (width & 1) || (height & 1) || fps < 1 || fps > 60 || bitrate < 100000 || bitrate > 50000000)
        return 2;
    signal(SIGTERM, stop_signal);
    signal(SIGINT, stop_signal);
    signal(SIGPIPE, SIG_IGN);
    av_log_set_level(AV_LOG_WARNING);
    avformat_network_init();

    int exit_code = 1;
    struct Capture camera = {.fd = -1};
    struct ScMemOpsS *memory = NULL;
    VideoEncoder *encoder = NULL;
    bool memory_open = false, encoder_initialized = false, buffers_allocated = false, mux_open = false;
    AVCodecContext *decoder = NULL;
    AVFrame *decoded = NULL;
    AVPacket *jpeg = NULL, *encoded = NULL;
    struct SwsContext *converter = NULL;
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
    settings.sProfileLevel.nLevel = VENC_H264Level4;
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
    parameters->level = 40;
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
    fprintf(stderr, "hwstream: Cedar VE2 hardware H.264 Baseline active, %d bps; monotonic RTP timestamps\n", bitrate);

    int64_t started = mono_us(), last_snapshot = 0, last_report = 0, last_pts = -1;
    uint64_t frames = 0, dropped = 0, timestamp_fallbacks = 0;
    while (!stopping) {
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
        av_packet_unref(jpeg);
        TRY(av_new_packet(jpeg, capture.bytesused));
        memcpy(jpeg->data, camera.buffers[capture.index].data, capture.bytesused);
        TRY(xioctl(camera.fd, VIDIOC_QBUF, &capture));
        if (avcodec_send_packet(decoder, jpeg) < 0 || avcodec_receive_frame(decoder, decoded) < 0) {
            avcodec_flush_buffers(decoder);
            ++dropped;
            continue;
        }
        if (decoded->width != width || decoded->height != height) {
            fprintf(stderr, "hwstream: JPEG dimensions changed unexpectedly\n");
            goto cleanup;
        }
        /* Cache by the original JPEG pixel format ourselves. sws_getCachedContext
         * compares against its internally normalized YUVJ format and otherwise
         * recreates the converter on every frame on this FFmpeg version.
         */
        if (!converter || conversion_format != decoded->format) {
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
        TRY(sws_scale(converter, (const uint8_t * const *)decoded->data, decoded->linesize,
                      0, height, planes, linesizes));
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
        if (now - last_snapshot >= 1000000) {
            snapshot(snapshot_path, jpeg->data, jpeg->size);
            last_snapshot = now;
        }
        if (now - last_report >= 1000000) {
            progress(frames, dropped, timestamp_fallbacks, pts, started, age_ms, encode_ms);
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
