#ifndef BOT_CAMERA_OUTPUT_H
#define BOT_CAMERA_OUTPUT_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

/* Limited-range NV12: Y=16..235, UV=16..240; neutral chroma must stay 128. */
struct CameraOutput {
    int contrast, saturation, black_point, gamma;
    int y_slope, y_offset, uv_slope;
    bool shoulder;
    uint8_t y_lut[256];
};
static int camera_full_to_limited_value(int value, bool chroma) {
    if (!chroma) return 16 + (value * 219 + 127) / 255;
    int delta=value-128, magnitude=delta<0?-delta:delta;
    int scaled=(magnitude * 224 + 127)/255;
    return 128 + (delta<0?-scaled:scaled);
}
static void camera_full_to_limited(unsigned char *pixels, int stride, int width, int height, bool chroma) {
    for (int y=0;y<height;++y) {
        unsigned char *row=pixels+(size_t)y*stride; int x=0;
#if defined(__aarch64__)
        for (;x+16<=width;x+=16) {
            uint8x16_t data=vld1q_u8(row+x);
            uint16x8_t parts[2]={vmovl_u8(vget_low_u8(data)),vmovl_u8(vget_high_u8(data))};
            for (int i=0;i<2;++i) {
                int16x8_t delta=vsubq_s16(vreinterpretq_s16_u16(parts[i]),vdupq_n_s16(128));
                uint16x8_t magnitude=chroma?vreinterpretq_u16_s16(vabsq_s16(delta)):parts[i];
                uint16x8_t n=vaddq_u16(vmulq_n_u16(magnitude,chroma?224:219),vdupq_n_u16(127));
                uint16x8_t q=vshrq_n_u16(vaddq_u16(vaddq_u16(n,vdupq_n_u16(1)),vshrq_n_u16(n,8)),8);
                if (chroma) {
                    int16x8_t signed_q=vreinterpretq_s16_u16(q);
                    signed_q=vbslq_s16(vcltq_s16(delta,vdupq_n_s16(0)),vnegq_s16(signed_q),signed_q);
                    parts[i]=vreinterpretq_u16_s16(vaddq_s16(vdupq_n_s16(128),signed_q));
                } else parts[i]=vaddq_u16(q,vdupq_n_u16(16));
            }
            vst1q_u8(row+x,vcombine_u8(vqmovn_u16(parts[0]),vqmovn_u16(parts[1])));
        }
#endif
        for (;x<width;++x) row[x]=(unsigned char)camera_full_to_limited_value(row[x],chroma);
    }
}
static int camera_output_value(const struct CameraOutput *t, int value, bool chroma) {
    int transformed = chroma ? value * t->uv_slope + 128 * (16 - t->uv_slope)
                             : value * t->y_slope + t->y_offset;
    if (!chroma && t->shoulder) {
        int shoulder = 205 * 16 + ((transformed - 205 * 16) * 3 + 4) / 8;
        if (transformed > shoulder) transformed = shoulder;
    }
    int result = transformed < 0 ? 0 : (transformed + 8) / 16;
    int maximum = chroma ? 240 : 235;
    return result < 16 ? 16 : result > maximum ? maximum : result;
}
static void camera_output_prepare(struct CameraOutput *t) {
    double contrast = t->contrast / 100.0;
    double slope = contrast * 219 / (219 - t->black_point);
    t->y_slope = (int)lround(slope * 16);
    t->y_offset = (int)lround((16 + 109.5 * (1 - contrast) - slope * (16 + t->black_point)) * 16);
    t->uv_slope = (int)lround(t->saturation * 16 / 100.0);
    t->shoulder = t->contrast > 100 || t->black_point > 0;
    for (int i = 0; i < 256; ++i) {
        int value = camera_output_value(t, i, false);
        if (t->gamma != 100) value = (int)lround(16 + 219 * pow((value - 16) / 219.0, 100.0 / t->gamma));
        t->y_lut[i] = (uint8_t)value;
    }
}
static void camera_output_default(struct CameraOutput *t) {
    /* Sensor CCM and calibrated gamma run in linear ISP space; no compensation hacks. */
    *t = (struct CameraOutput){.contrast=100, .saturation=100, .black_point=0, .gamma=100};
    camera_output_prepare(t);
}
static void camera_output_correct(const struct CameraOutput *t, unsigned char *pixels,
                                  int stride, int width, int height, bool chroma,
                                  const unsigned char *source, int source_stride) {
    for (int y = 0; y < height; ++y) {
        unsigned char *row = pixels + (size_t)y * stride;
        const unsigned char *in = source + (size_t)y * source_stride;
        int x = 0;
#if defined(__aarch64__)
        if (chroma || t->gamma == 100) {
            int slope = chroma ? t->uv_slope : t->y_slope;
            int offset = chroma ? 128 * (16 - slope) : t->y_offset;
            for (; x + 16 <= width; x += 16) {
                uint8x16_t bytes = vld1q_u8(in + x);
                int16x8_t lo = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(bytes)));
                int16x8_t hi = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(bytes)));
                lo = vmlaq_n_s16(vdupq_n_s16(offset), lo, slope);
                hi = vmlaq_n_s16(vdupq_n_s16(offset), hi, slope);
                if (!chroma && t->shoulder) {
                    int16x8_t s1 = vaddq_s16(vdupq_n_s16(3280), vrshrq_n_s16(vmulq_n_s16(vsubq_s16(lo, vdupq_n_s16(3280)), 3), 3));
                    int16x8_t s2 = vaddq_s16(vdupq_n_s16(3280), vrshrq_n_s16(vmulq_n_s16(vsubq_s16(hi, vdupq_n_s16(3280)), 3), 3));
                    lo = vminq_s16(lo, s1); hi = vminq_s16(hi, s2);
                }
                uint8x16_t out = vcombine_u8(vqrshrun_n_s16(lo, 4), vqrshrun_n_s16(hi, 4));
                out = vmaxq_u8(vdupq_n_u8(16), vminq_u8(vdupq_n_u8(chroma ? 240 : 235), out));
                vst1q_u8(row + x, out);
            }
        }
#endif
        for (; x < width; ++x) row[x] = chroma ? camera_output_value(t, in[x], true) : t->y_lut[in[x]];
    }
}
#endif
