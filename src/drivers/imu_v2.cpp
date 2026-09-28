#include "imu_v2.hpp"
#include "bmi270.hpp"
#include "../config/airframe.hpp"
#include "../core/imu_selection.hpp"
#include "../estimation/imu_rotation.hpp"
#include "../estimation/ahrs.hpp"
#include <cmath>
namespace imu_v2 {
namespace {
imu270::Device device[2];
estimation::ImuPrep prep[2];
estimation::ImuSample sample[2]{};
bool ready[2]={false,false},have[2]={false,false};
uint32_t last_us[2]{};
core::ImuSelection selection;
bool started=false;
Observer observer=nullptr; uint32_t frame_sequence[2]{};
bool differs() {
    const auto& a=sample[0]; const auto& b=sample[1];
    const float gx=a.gx_dps-b.gx_dps,gy=a.gy_dps-b.gy_dps,gz=a.gz_dps-b.gz_dps;
    const float ax=a.ax_g-b.ax_g,ay=a.ay_g-b.ay_g,az=a.az_g-b.az_g;
    return gx*gx+gy*gy+gz*gz>400 || ax*ax+ay*ay+az*az>0.25f;
}
}
void set_observer(Observer cb) { observer=cb; }
bool begin() {
    selection={}; ahrs::reset(); started=false;
    for(unsigned i=0;i<2;++i) {
        frame_sequence[i]=0;
        prep[i]={}; prep[i].configure(400,config::gyro_lpf_hz,config::accel_lpf_hz,4);
        prep[i].configure_notch(400,config::gyro_notch_hz,config::gyro_notch_q); have[i]=false;
        ready[i]=estimation::rotation_valid(config::imu_rotation[i]) &&
            device[i].begin(i?hal::SpiBus::imu2:hal::SpiBus::imu,i?hal::pins::imu2_cs:hal::pins::imu_cs);
    }
    return ready[0]||ready[1];
}
bool sensor_healthy(unsigned i) { return i<2 && ready[i] && have[i] && uint32_t(hal::micros()-last_us[i])<20000; }
void poll(bool allow_calibration) {
    if(!started) {
        // Other sensor/storage initialization can take longer than a FIFO.
        // Flush once before sampling, never to conceal an in-flight backlog.
        for(unsigned i=0;i<2;++i) if(ready[i]) ready[i]=device[i].discard_pending();
        started=true; return;
    }
    estimation::ImuSample processed[2][8]{};
    uint8_t counts[2]{};
    for(unsigned i=0;i<2;++i) {
        if(!ready[i]) continue;
        imu270::Sample raw[8];
        const auto result=device[i].read(raw,counts[i]);
        if(result==imu270::Result::fault) { ready[i]=false; continue; }
        if(result!=imu270::Result::ok) continue;
        const uint32_t batch_time=hal::micros();
        for(unsigned n=0;n<counts[i];++n) {
            const float acc[3]={raw[n].ax_g,raw[n].ay_g,raw[n].az_g};
            const float gyr[3]={raw[n].gx_dps,raw[n].gy_dps,raw[n].gz_dps};
            float a[3],g[3];
            estimation::rotate(config::imu_rotation[i],acc,a);
            estimation::rotate(config::imu_rotation[i],gyr,g);
            // Bosch acceleration is specific force. AHRS uses its negative.
            const float values[6]={g[0],g[1],g[2],-a[0],-a[1],-a[2]};
            for(unsigned axis=0;axis<3;++axis)
                a[axis]=(-a[axis]-config::accel_offset_g[i][axis])*config::accel_scale[i][axis];
            processed[i][n]=prep[i].process(g[0],g[1],g[2],a[0],a[1],a[2],0.0025f,allow_calibration);
            // Reconstructed acquisition time: nominal FIFO spacing, anchored
            // at retrieval. Not hardware synchronized between the two IMUs.
            const uint32_t stamp=batch_time-(counts[i]-1-n)*2500;
            if(observer) observer(i,stamp,frame_sequence[i],device[i].sensor_time(),values,processed[i][n]);
            ++frame_sequence[i];
        }
        sample[i]=processed[i][counts[i]-1]; last_us[i]=hal::micros(); have[i]=true;
    }
    const bool h0=sensor_healthy(0), h1=sensor_healthy(1);
    const unsigned previous=selection.active();
    selection.update(h0,h1 && prep[1].bias_ready(),
        prep[0].bias_ready()&&prep[1].bias_ready()&&differs(),hal::millis());
    const unsigned selected=selection.active();
    if(selected!=previous) ahrs::clear_residual_bias();
    if(selection.healthy()) for(unsigned n=0;n<counts[selected];++n) {
        const auto& p=processed[selected][n];
        ahrs::update(p.ax_g,p.ay_g,p.az_g,p.gx_dps,p.gy_dps,p.gz_dps,0.0025f);
    }
}
bool healthy() { return selection.healthy() && sensor_healthy(selection.active()) && ahrs::valid() && config::imu_orientation_confirmed; }
bool bias_ready() { return prep[selection.active()].bias_ready(); }
bool ambiguous() { return selection.ambiguous(); }
unsigned active() { return selection.active(); }
const estimation::ImuSample& latest() { return sample[selection.active()]; }
}
