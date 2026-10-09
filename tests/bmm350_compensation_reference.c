/* Test oracle: compile the unmodified Bosch implementation in this TU so its
 * internal OTP decoder can be exercised without copying its implementation. */
#include "../lib/bmm350/src/bmm350.c"
#include <string.h>
static int8_t read_sample(uint8_t reg,uint8_t *out,uint32_t len,void *ctx) {
    if(reg!=BMM350_REG_MAG_X_XLSB||len!=14) return -1;
    const int32_t *raw=ctx; out[0]=0xde;out[1]=0xad;
    for(unsigned i=0;i<4;++i)for(unsigned b=0;b<3;++b)out[2+i*3+b]=(uint32_t)raw[i]>>(8*b);
    return 0;
}
static int8_t unused_write(uint8_t reg,const uint8_t *data,uint32_t len,void *ctx) {
    (void)reg;(void)data;(void)len;(void)ctx;return -1;
}
static void unused_delay(uint32_t us,void *ctx) {(void)us;(void)ctx;}
int reference_compensation(const uint16_t *otp,const int32_t *raw,float *out,float *trim) {
    struct bmm350_dev dev={0};
    dev.read=read_sample;dev.write=unused_write;dev.delay_us=unused_delay;
    dev.intf_ptr=(void*)raw;dev.axis_en=7;
    memcpy(dev.otp_data,otp,sizeof(dev.otp_data));
    update_mag_off_sens(&dev);
    _Static_assert(sizeof(dev.mag_comp)==19*sizeof(float),"Unexpected Bosch trim layout");
    memcpy(trim,&dev.mag_comp,sizeof(dev.mag_comp));
    struct bmm350_mag_temp_data sample={0};
    const int result=bmm350_get_compensated_mag_xyz_temp_data(&sample,&dev);
    out[0]=sample.x;out[1]=sample.y;out[2]=sample.z;out[3]=sample.temperature;
    return result;
}
