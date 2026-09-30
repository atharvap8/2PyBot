#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vencoder.h>
#include <memoryAdapter.h>

#define CHECK(call) do { int result = (call); if (result != 0) { \
    fprintf(stderr, "%s failed: %d\n", #call, result); exit(1); } } while (0)

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    struct ScMemOpsS *memory = MemAdapterGetOpsS();
    if (!memory) return 3;
    CHECK(CdcMemOpen(memory));
    VideoEncoder *encoder = VideoEncCreate(VENC_CODEC_H264);
    if (!encoder) return 4;
    VencH264Param settings = {0};
    settings.sProfileLevel.nProfile = VENC_H264ProfileBaseline;
    settings.sProfileLevel.nLevel = VENC_H264Level4;
    settings.sQPRange.nMinqp = 15;
    settings.sQPRange.nMaxqp = 42;
    settings.sQPRange.nMinPqp = 15;
    settings.sQPRange.nMaxPqp = 42;
    settings.sQPRange.nQpInit = 28;
    settings.nFramerate = settings.nSrcFramerate = 30;
    settings.nBitrate = 6000000;
    settings.nMaxKeyInterval = 15;
    settings.nCodingMode = VENC_FRAME_CODING;
    settings.sRcParam.eRcMode = AW_CBR;
    CHECK(VideoEncSetParameter(encoder, VENC_IndexParamH264Param, &settings));
    VencBaseConfig base = {0};
    base.nInputWidth = base.nDstWidth = base.nStride = 1920;
    base.nInputHeight = base.nDstHeight = 1080;
    base.eInputFormat = VENC_PIXEL_YUV420SP;
    base.memops = memory;
    CHECK(VideoEncInit(encoder, &base));
    VencAllocateBufferParam allocation = {.nBufferNum=1, .nSizeY=1920*1088, .nSizeC=1920*544};
    CHECK(AllocInputBuffer(encoder, &allocation));
    VencHeaderData header = {0};
    CHECK(VideoEncGetParameter(encoder, VENC_IndexParamH264SPSPPS, &header));
    FILE *output = fopen(argv[1], "wb");
    if (!output) return 5;
    fwrite(header.pBuffer, 1, header.nLength, output);
    for (int i = 0; i < 30; ++i) {
        VencInputBuffer input = {0};
        CHECK(GetOneAllocInputBuffer(encoder, &input));
        memset(input.pAddrVirY, 32 + i * 4, allocation.nSizeY);
        memset(input.pAddrVirC, 128, allocation.nSizeC);
        input.nPts = (long long)i * 1000000 / 30;
        input.nWidth = 1920;
        input.nHeight = 1080;
        CHECK(FlushCacheAllocInputBuffer(encoder, &input));
        CHECK(AddOneInputBuffer(encoder, &input));
        CHECK(VideoEncodeOneFrame(encoder));
        CHECK(AlreadyUsedInputBuffer(encoder, &input));
        CHECK(ReturnOneAllocInputBuffer(encoder, &input));
        VencOutputBuffer packet = {0};
        CHECK(GetOneBitstreamFrame(encoder, &packet));
        fwrite(packet.pData0, 1, packet.nSize0, output);
        if (packet.nSize1) fwrite(packet.pData1, 1, packet.nSize1, output);
        fprintf(stderr, "frame=%d bytes=%u pts=%lld\n", i, packet.nSize0 + packet.nSize1, packet.nPts);
        CHECK(FreeOneBitStreamFrame(encoder, &packet));
    }
    fclose(output);
    CHECK(ReleaseAllocInputBuffer(encoder));
    CHECK(VideoEncUnInit(encoder));
    VideoEncDestroy(encoder);
    CdcMemClose(memory);
    return 0;
}
