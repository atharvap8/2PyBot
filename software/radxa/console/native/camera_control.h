#ifndef BOT_CAMERA_CONTROL_H
#define BOT_CAMERA_CONTROL_H
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

/* Sensor/ISP writes run in the process owning the capture descriptor. */
static int camera_set_v4l(struct Capture *c, uint32_t id, int value) {
    struct v4l2_control control = {.id=id, .value=value};
    return xioctl(c->fd, VIDIOC_S_CTRL, &control);
}
#include "imx708_profile.h"
static double camera_line_us(const struct Capture *c) {
    return c->width == 1536 ? 5216.0 / 566.4 : c->width == 4608 ? 15648.0 / 595.2 : 7824.0 / 585.6;
}
static int camera_max_exposure(const struct Capture *c) {
    int lines = c->width == 1536 ? 904 : c->width == 4608 ? 2650 : 1336;
    return (int)lround((lines - 48) * camera_line_us(c));
}
static int camera_sensor_word(struct Capture *c, uint16_t reg) {
    if (c->sensor_fd < 0) return -1;
    unsigned char address[2] = {reg >> 8, reg & 255}, value[2] = {0};
    struct i2c_msg messages[2] = {{.addr=0x1a, .len=2, .buf=address},
        {.addr=0x1a, .flags=I2C_M_RD, .len=2, .buf=value}};
    struct i2c_rdwr_ioctl_data data = {.msgs=messages, .nmsgs=2};
    if (ioctl(c->sensor_fd, I2C_RDWR, &data) != 2) return -1;
    return (value[0] << 8) | value[1];
}
static int camera_actual_exposure(struct Capture *c) {
    int lines = camera_sensor_word(c, 0x0202);
    return lines < 0 ? c->manual_exposure : (int)lround(lines * camera_line_us(c));
}
static int camera_actual_gain(struct Capture *c) {
    int code = camera_sensor_word(c, 0x0204);
    return code < 0 || code >= 1024 ? c->manual_gain : (int)lround(16384.0 / (1024 - code));
}
static int camera_set_auto(struct Capture *c, int mode) {
    if (camera_set_v4l(c, V4L2_CID_EXPOSURE_AUTO, mode) < 0) return -1;
    if (camera_set_v4l(c, V4L2_CID_AUTOGAIN, mode == 0 ? 1 : 0) < 0) return -1;
    c->exposure_auto = mode;
    return 0;
}
static int camera_control_set(struct Capture *c, const char *name, int value) {
    if (!strcmp(name,"color_temperature")) {
        if (value<2964 || value>7590) return -1;
        c->wb_auto=false;
        return imx708_set_temperature(c,value);
    }
    if (!strcmp(name,"exposure_auto")) {
        if (value < 0 || value > 1) return -1;
        return camera_set_auto(c,value);
    }
    if (!strcmp(name,"exposure_us")) {
        if (value < 54 || value > camera_max_exposure(c)) return -1;
        if (camera_set_auto(c,1) < 0 || camera_set_v4l(c,V4L2_CID_EXPOSURE_ABSOLUTE,value) < 0) return -1;
        c->manual_exposure=value; return 0;
    }
    if (!strcmp(name,"gain")) {
        if (value < 16 || value > 256) return -1;
        /* ISP manual sensor_gain is Q8; the sensor driver/readback is Q4. */
        if (camera_set_auto(c,1) < 0 || camera_set_v4l(c,V4L2_CID_GAIN,value * 16) < 0) return -1;
        c->manual_gain=value; return 0;
    }
    if (!strcmp(name,"white_balance_auto")) {
        if (value < 0 || value > 1) return -1;
        c->wb_auto=value; return 0;
    }
    if (!strcmp(name,"red_balance") || !strcmp(name,"blue_balance")) {
        if (value < 64 || value > 4095 || !c->wb_manual) return -1;
        c->wb_gains[!strcmp(name,"red_balance") ? 0 : 3]=value; c->wb_auto=false;
        if (isp_set_attr_cfg(c->isp_id,ISP_CTRL_WB_MGAIN,(int32_t*)c->wb_gains)) return -1;
        return imx708_apply_ccm(c,imx708_temperature_from_gains(c->wb_gains));
    }
    int *setting=NULL, low=0, high=0;
    if (!strcmp(name,"contrast")) {setting=&c->output.contrast; low=80; high=180;}
    else if (!strcmp(name,"saturation")) {setting=&c->output.saturation; low=0; high=200;}
    else if (!strcmp(name,"black_point")) {setting=&c->output.black_point; low=0; high=48;}
    else if (!strcmp(name,"gamma")) {setting=&c->output.gamma; low=70; high=160;}
    if (!setting || value < low || value > high) return -1;
    *setting=value; camera_output_prepare(&c->output); return 0;
}
static void camera_control_status(struct Capture *c, char *text, size_t length) {
    int exp=camera_actual_exposure(c), gain=camera_actual_gain(c);
    snprintf(text,length,
        "{\"ok\":true,\"backend\":\"imx708-isp\",\"colour_profile\":\"rpi-imx708\",\"ccm_enabled\":%s,\"exposure_max_us\":%d,"
        "\"values\":{\"exposure_auto\":%d,\"exposure_us\":%d,\"gain\":%d,"
        "\"white_balance_auto\":%d,\"red_balance\":%u,\"blue_balance\":%u,"
        "\"contrast\":%d,\"saturation\":%d,\"black_point\":%d,\"gamma\":%d,\"color_temperature\":%d}}",
        c->ccm_enabled?"true":"false",camera_max_exposure(c),c->exposure_auto,exp,gain,c->wb_auto,c->wb_gains[0],c->wb_gains[3],
        c->output.contrast,c->output.saturation,c->output.black_point,c->output.gamma,c->color_temperature);
}
static int camera_control_open(struct Capture *c) {
    if (!c->nv12) return 0;
    const char *path=getenv("BOT_CAM_SOCKET");
    if (!path) path="/dev/shm/2pybot-camera.sock";
    if (strlen(path) >= sizeof(c->control_path)) return -1;
    strcpy(c->control_path,path);
    c->control_fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if (c->control_fd<0) return -1;
    struct sockaddr_un address={.sun_family=AF_UNIX}; strcpy(address.sun_path,path);
    unlink(path);
    if (bind(c->control_fd,(struct sockaddr*)&address,sizeof(address))<0) return -1;
    chmod(path,0600);
    c->sensor_fd=open("/dev/i2c-3",O_RDWR|O_CLOEXEC);
    fprintf(stderr,"hwstream: live sensor/ISP controls on %s; exposure units microseconds\n",path);
    return 0;
}
static void camera_isp_diagnostics(struct Capture *c, char *text, size_t length) {
    unsigned char buffer[65536]={0}, modules[256]={0};
    int used=snprintf(text,length,"{\"ok\":true,\"ccm\":[");
    for (int slot=0;slot<3;++slot) {
        int bytes=isp_get_cfg(c->isp_id,3,0x80U<<slot,buffer);
        uint16_t temp=0; int16_t values[12]={0};
        memcpy(&temp,buffer,2); memcpy(values,buffer+2,24);
        used+=snprintf(text+used,length-used,"%s{\"bytes\":%d,\"temp\":%u,\"values\":[",slot?",":"",bytes,temp);
        for (int i=0;i<12;++i) used+=snprintf(text+used,length-used,"%s%d",i?",":"",values[i]);
        used+=snprintf(text+used,length-used,"]}");
    }
    int bytes=isp_get_cfg(c->isp_id,4,2,buffer), number=0;
    memcpy(&number,buffer,4);
    used+=snprintf(text+used,length-used,"],\"gamma\":{\"bytes\":%d,\"number\":%d,\"samples\":[",bytes,number);
    for (int i=0;i<9;++i) {
        uint16_t value=0; memcpy(&value,buffer+4+2*(i*1023/8),2);
        used+=snprintf(text+used,length-used,"%s%u",i?",":"",value);
    }
    int module_bytes=isp_get_cfg(c->isp_id,1,0x20,modules);
    used+=snprintf(text+used,length-used,"]},\"module_bytes\":%d,\"modules\":[",module_bytes);
    for (int i=0;i<33;++i) used+=snprintf(text+used,length-used,"%s%u",i?",":"",modules[i]);
    int32_t color_space=-1;
    isp_get_attr_cfg(c->isp_id,17,&color_space);
    struct v4l2_format format={.type=c->type};
    xioctl(c->fd,VIDIOC_G_FMT,&format);
    snprintf(text+used,length-used,"],\"color_space\":%d,\"capture_colorspace\":%u,\"capture_quantization\":%u}",
             color_space,format.fmt.pix_mp.colorspace,format.fmt.pix_mp.quantization);
}
static void camera_control_poll(struct Capture *c) {
    if (c->control_fd<0) return;
    for (int i=0;i<4;++i) {
        char request[192]={0}, response[2048]={0};
        struct sockaddr_un sender={0}; socklen_t sender_length=sizeof(sender);
        ssize_t count=recvfrom(c->control_fd,request,sizeof(request)-1,MSG_DONTWAIT,
                               (struct sockaddr*)&sender,&sender_length);
        if (count<0) break;
        if (!strcmp(request,"GET")) camera_control_status(c,response,sizeof(response));
        else if (!strcmp(request,"ISP_PROFILE")) camera_isp_diagnostics(c,response,sizeof(response));
        else {
            char name[40]={0},extra=0; int value=0;
            if (sscanf(request,"SET %39[a-z_] %d%c",name,&value,&extra)!=2 || camera_control_set(c,name,value)<0)
                snprintf(response,sizeof(response),"{\"ok\":false,\"error\":\"Invalid, out-of-range or unsupported camera control\"}");
            else camera_control_status(c,response,sizeof(response));
        }
        sendto(c->control_fd,response,strlen(response),MSG_DONTWAIT,(struct sockaddr*)&sender,sender_length);
    }
}
static void camera_control_close(struct Capture *c) {
    if (c->control_fd>=0) {close(c->control_fd);unlink(c->control_path);c->control_fd=-1;}
    if (c->sensor_fd>=0) {close(c->sensor_fd);c->sensor_fd=-1;}
}
#endif
