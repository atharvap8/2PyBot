/*
 * awnn_runner: persistent NPU inference coprocess for the console detector.
 *
 * Loads an ACUITY-compiled NBG once through Radxa's awnn/VIPLite helper, then
 * serves repeated inferences over pipes so the network stays resident (init is
 * paid once, each inference is a few ms). It reuses the vendor awnn_lib.c, which
 * logs to stdout via printf; to keep a clean binary channel we redirect fd 1 to
 * stderr at startup and send results on a saved copy of the original stdout.
 *
 * Protocol (all on the saved stdout fd):
 *   on start:  one text line  "READY insz=<bytes> outs=<n> <elem0> <elem1>...\n"
 *   per call:  read insz bytes (the input tensor) from stdin, then write each
 *              output tensor back as raw float32 (<elemK>*4 bytes), in order.
 * Exit cleanly on stdin EOF.
 *
 * Build: scripts/vision/npu_build_runner.sh  (needs Radxa's ai-sdk checkout).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "awnn_internal.h"   /* struct Awnn_Context_t: input/output counts + elements */
#include "awnn_lib.h"

static int read_full(int fd, void *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, (char *)buf + got, n - got);
        if (r <= 0) return 0;
        got += (size_t)r;
    }
    return 1;
}

static int write_full(int fd, const void *buf, size_t n) {
    size_t put = 0;
    while (put < n) {
        ssize_t w = write(fd, (const char *)buf + put, n - put);
        if (w <= 0) return 0;
        put += (size_t)w;
    }
    return 1;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: awnn_runner model.nb\n");
        return 2;
    }
    /* Clean binary channel: results go to a saved copy of stdout; redirect fd 1
     * (where awnn/driver printf noise lands) to stderr. */
    int out = dup(1);
    dup2(2, 1);

    awnn_init();
    Awnn_Context_t *ctx = awnn_create(argv[1]);
    if (!ctx) {
        fprintf(stderr, "awnn_runner: create failed for %s\n", argv[1]);
        return 3;
    }
    unsigned int insz = ctx->input_params[0].elements;   /* uint8 input: 1 byte/elem */
    int outs = (int)ctx->output_count;

    char header[512];
    int hlen = snprintf(header, sizeof(header), "READY insz=%u outs=%d", insz, outs);
    for (int i = 0; i < outs; i++)
        hlen += snprintf(header + hlen, sizeof(header) - hlen, " %u", ctx->output_params[i].elements);
    hlen += snprintf(header + hlen, sizeof(header) - hlen, "\n");
    if (!write_full(out, header, (size_t)hlen)) return 4;

    unsigned char *inbuf = (unsigned char *)calloc(1, insz + 4096);
    void *inbufs[1] = { inbuf };
    while (read_full(0, inbuf, insz)) {
        awnn_set_input_buffers(ctx, inbufs);
        awnn_run(ctx);
        float **res = awnn_get_output_buffers(ctx);
        int ok = 1;
        for (int i = 0; i < outs && ok; i++)
            ok = write_full(out, res[i], (size_t)ctx->output_params[i].elements * sizeof(float));
        if (!ok) break;
    }
    free(inbuf);
    awnn_destroy(ctx);
    awnn_uninit();
    return 0;
}
