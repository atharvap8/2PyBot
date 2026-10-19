#define IMX708_PROFILE_MATH_ONLY
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../software/radxa/console/native/imx708_profile.h"

int main(void) {
    for (int temperature=2964;temperature<=7590;temperature+=100) {
        unsigned char payload[26]; int16_t values[12]; double matrix[9],r,b;
        imx708_ccm_payload(temperature,5000,payload);
        memcpy(values,payload+2,24);
        for (int row=0;row<3;++row) assert(values[row*3]+values[row*3+1]+values[row*3+2]==256);
        for (int i=9;i<12;++i) assert(values[i]==0);
        imx708_interpolate(temperature,matrix,&r,&b);
        uint16_t gains[4]={(uint16_t)lround(256/r),256,256,(uint16_t)lround(256/b)};
        assert(abs(imx708_temperature_from_gains(gains)-temperature)<40);
    }
    assert(imx708_gamma_value(0)==0 && imx708_gamma_value(1023)==4095);
    for (int i=1;i<1024;++i) assert(imx708_gamma_value(i)>=imx708_gamma_value(i-1));
    puts("IMX708 profile: Q8 neutral preservation, CCT interpolation, AWB locus and gamma endpoints/monotonicity passed.");
    return 0;
}
