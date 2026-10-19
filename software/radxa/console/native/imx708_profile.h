#ifndef BOT_IMX708_PROFILE_H
#define BOT_IMX708_PROFILE_H
#include <math.h>
#include <stdint.h>
#include <string.h>

/* Raspberry Pi libcamera IMX708 calibration, commit
 * 9f4754cbc6f9ae2276c9e2f42db9302e64c723b5. Numerical data only;
 * ISP602 packing, interpolation and runtime application are implemented here.
 */
struct Imx708Calibration {int temperature; double r_ratio,b_ratio; double matrix[9];};
static const struct Imx708Calibration imx708_calibration[] = {
 {2964,.7451,.3213,{1.72129,-.45961,-.26169,-.30042,1.56924,-.26882,.15133,-1.13293,1.98161}},
 {3610,.6119,.4443,{1.54474,-.35082,-.19391,-.36989,1.67926,-.30936,-.00524,-.55197,1.55722}},
 {4640,.5168,.5419,{1.52972,-.35168,-.17804,-.28309,1.67098,-.38788,.01695,-.57209,1.55515}},
 {5910,.4436,.6229,{1.56879,-.42159,-.14719,-.27275,1.59354,-.32079,-.02862,-.40662,1.43525}},
 {7590,.3847,.6921,{1.41424,-.21092,-.20332,-.17646,1.71734,-.54087,.01297,-.63111,1.61814}}
};
static const uint16_t imx708_gamma_points[][2] = {
 {0,0},{1024,5040},{2048,9338},{3072,12356},{4096,15312},{5120,18051},{6144,20790},
 {7168,23193},{8192,25744},{9216,27942},{10240,30035},{11264,32005},{12288,33975},
 {13312,35815},{14336,37600},{15360,39168},{16384,40642},{18432,43379},{20480,45749},
 {22528,47753},{24576,49621},{26624,51253},{28672,52698},{30720,53796},{32768,54876},
 {36864,57012},{40960,58656},{45056,59954},{49152,61183},{53248,62355},{57344,63419},
 {61440,64476},{65535,65535}
};

static void imx708_interpolate(int temperature, double matrix[9], double *r, double *b) {
    int index=0;
    while (index<3 && temperature>imx708_calibration[index+1].temperature) ++index;
    const struct Imx708Calibration *lo=&imx708_calibration[index], *hi=lo+1;
    double t=(temperature-lo->temperature)/(double)(hi->temperature-lo->temperature);
    if (t<0) t=0;
    if (t>1) t=1;
    for (int i=0;i<9;++i) matrix[i]=lo->matrix[i]*(1-t)+hi->matrix[i]*t;
    *r=lo->r_ratio*(1-t)+hi->r_ratio*t;
    *b=lo->b_ratio*(1-t)+hi->b_ratio*t;
}
static void imx708_ccm_payload(int temperature, int trigger, unsigned char payload[26]) {
    double matrix[9],r,b; int16_t values[12]={0}; uint16_t ct=trigger;
    imx708_interpolate(temperature,matrix,&r,&b);
    for (int row=0;row<3;++row) {
        values[row*3]=(int16_t)lround(matrix[row*3]*256);
        values[row*3+1]=(int16_t)lround(matrix[row*3+1]*256);
        values[row*3+2]=256-values[row*3]-values[row*3+1]; /* Exact neutral preservation. */
    }
    memcpy(payload,&ct,2); memcpy(payload+2,values,24);
}
static uint16_t imx708_gamma_value(int index) {
    double x=index*65535.0/1023; int point=0;
    while (point<31 && x>imx708_gamma_points[point+1][0]) ++point;
    double t=(x-imx708_gamma_points[point][0])/(imx708_gamma_points[point+1][0]-imx708_gamma_points[point][0]);
    double y=imx708_gamma_points[point][1]*(1-t)+imx708_gamma_points[point+1][1]*t;
    return (uint16_t)lround(y*4095/65535);
}
static int imx708_temperature_from_gains(const uint16_t gains[4]) {
    double green=(gains[1]+gains[2])/2.0;
    if (!gains[0] || !gains[3]) return 5000;
    double r=green/gains[0], b=green/gains[3], best=1e9; int chosen=5000;
    for (int i=0;i<4;++i) {
        const struct Imx708Calibration *lo=&imx708_calibration[i], *hi=lo+1;
        double dr=hi->r_ratio-lo->r_ratio, db=hi->b_ratio-lo->b_ratio;
        double t=((r-lo->r_ratio)*dr+(b-lo->b_ratio)*db)/(dr*dr+db*db);
        if (t<0) t=0;
        if (t>1) t=1;
        double error=pow(r-lo->r_ratio-t*dr,2)+pow(b-lo->b_ratio-t*db,2);
        if (error<best) {best=error;chosen=(int)lround(lo->temperature+t*(hi->temperature-lo->temperature));}
    }
    return chosen;
}

#ifndef IMX708_PROFILE_MATH_ONLY
static int imx708_apply_ccm(struct Capture *c, int temperature) {
    const int triggers[3]={2964,4640,7590}; unsigned char payload[26],readback[64];
    for (int slot=0;slot<3;++slot) {
        imx708_ccm_payload(temperature,triggers[slot],payload);
        if (isp_set_cfg(c->isp_id,3,0x80U<<slot,payload)!=26 ||
            isp_get_cfg(c->isp_id,3,0x80U<<slot,readback)!=26 || memcmp(payload,readback,26)) return -1;
    }
    if (isp_update(c->isp_id)) return -1;
    c->color_temperature=temperature;
    return 0;
}
static int imx708_set_temperature(struct Capture *c, int temperature) {
    double matrix[9],r,b;
    if (temperature<2500 || temperature>8000) return -1;
    imx708_interpolate(temperature,matrix,&r,&b);
    c->wb_gains[0]=(uint16_t)lround(256/r); c->wb_gains[1]=c->wb_gains[2]=256;
    c->wb_gains[3]=(uint16_t)lround(256/b);
    if (camera_set_v4l(c,V4L2_CID_AUTO_WHITE_BALANCE,0)<0 ||
        isp_set_attr_cfg(c->isp_id,ISP_CTRL_WB_MGAIN,(int32_t*)c->wb_gains)) return -1;
    c->wb_manual=true;
    return imx708_apply_ccm(c,temperature);
}
static int imx708_install_profile(struct Capture *c) {
    unsigned char gamma[32768]={0},modules[256]={0};
    int bytes=isp_get_cfg(c->isp_id,4,2,gamma), number=0;
    memcpy(&number,gamma,4);
    if (bytes!=30736 || number<1 || number>5 || isp_get_cfg(c->isp_id,1,0x20,modules)!=33) return -1;
    for (int table=0;table<5;++table) for (int channel=0;channel<3;++channel)
        for (int i=0;i<1024;++i) {
            uint16_t value=imx708_gamma_value(i);
            memcpy(gamma+4+2*(table*3072+channel*1024+i),&value,2);
        }
    if (isp_set_cfg(c->isp_id,4,2,gamma)!=bytes) return -1;
    modules[17]=modules[18]=modules[19]=modules[24]=modules[26]=modules[27]=modules[29]=0;
    modules[20]=modules[23]=modules[28]=1; /* Raw demosaic, sensor CCM, calibrated gamma. */
    if (isp_set_cfg(c->isp_id,1,0x20,modules)!=33 || isp_update(c->isp_id)) return -1;
    c->ccm_enabled=true;
    int32_t space=-1;
    if (isp_get_attr_cfg(c->isp_id,17,&space)) return -1;
    /* Installed RGB2YUV table: indices 0..2 are full range; 3..5 limited. */
    if (space<0 || space>5) return -1;
    c->input_full_range=space<3;
    fprintf(stderr,"hwstream: ISP RGB2YUV space=%d, %s -> limited-range H.264\n",space,c->input_full_range?"full range converted":"already limited");
    fprintf(stderr,"hwstream: Raspberry Pi IMX708 CCM/gamma profile installed in ISP602 before YUV output\n");
    return imx708_set_temperature(c,5000);
}
#endif
#endif
