#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../software/radxa/console/native/camera_output.h"

int main(void) {
    struct CameraOutput t;
    assert(camera_full_to_limited_value(0,false)==16);
    assert(camera_full_to_limited_value(255,false)==235);
    assert(camera_full_to_limited_value(128,true)==128);
    for (int chroma=0;chroma<2;++chroma) {
        unsigned char ramp[256];
        for (int i=0;i<256;++i) ramp[i]=i;
        camera_full_to_limited(ramp,256,256,1,chroma);
        for (int i=0;i<256;++i) assert(ramp[i]==camera_full_to_limited_value(i,chroma));
    }
    camera_output_default(&t);
    for (int saturation=0;saturation<=200;saturation+=50) {
        t.saturation=saturation; camera_output_prepare(&t);
        assert(camera_output_value(&t,128,true)==128);
    }
    t=(struct CameraOutput){.contrast=100,.saturation=100,.black_point=0,.gamma=100};
    camera_output_prepare(&t);
    for (int x=16;x<=235;++x) assert(t.y_lut[x]==x);
    for (int gamma=70;gamma<=160;gamma+=30) {
        for (int contrast=80;contrast<=180;contrast+=25) {
            for (int black=0;black<=48;black+=16) {
                t=(struct CameraOutput){.contrast=contrast,.saturation=150,.black_point=black,.gamma=gamma};
                camera_output_prepare(&t);
                for (int x=0;x<256;++x) {
                    assert(t.y_lut[x]>=16 && t.y_lut[x]<=235);
                    if (x) assert(t.y_lut[x]>=t.y_lut[x-1]);
                }
                for (int chroma=0;chroma<=1;++chroma) {
                    unsigned char pixels[69],original[69];
                    memset(pixels,77,sizeof(pixels));
                    for (int y=0;y<3;++y) for (int x=0;x<19;++x) pixels[y*23+x]=(x*13+y*7)&255;
                    memcpy(original,pixels,sizeof(pixels));
                    camera_output_correct(&t,pixels,23,19,3,chroma,pixels,23);
                    for (int y=0;y<3;++y) {
                        for (int x=0;x<19;++x) assert(pixels[y*23+x]==(chroma ? camera_output_value(&t,original[y*23+x],true) : t.y_lut[original[y*23+x]]));
                        for (int x=19;x<23;++x) assert(pixels[y*23+x]==77);
                    }
                }
            }
        }
    }
    puts("Camera output: neutral chroma, legal range, monotonic tones, SIMD/scalar parity and row padding passed.");
    return 0;
}
