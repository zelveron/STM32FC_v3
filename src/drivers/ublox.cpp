#include "ublox.hpp"
#include "../hal/hal.hpp"
#include <cmath>
#include <cstdlib>
#include <cstring>
namespace ublox {
namespace {
constexpr hal::Uart kPort=hal::Uart::gps;
constexpr uint32_t kBauds[]={38400,9600,115200,57600,19200};
uint32_t baud_now=38400, baud_since=0, message_ms=0, fix_ms=0, speed_ms=0;
unsigned baud_index=0;
bool have_message=false, have_fix=false, have_speed=false, collecting=false;
uint32_t bytes=0,fix_seq=0;
char fix_utc[16]{};
uint8_t boot[160]{};
uint16_t boot_len=0;
char line[128]{}, last_line[128]{}, utc[12]{};
size_t length=0;
int quality=0, satellites=0;
double latitude=0, longitude=0;
float altitude=0, speed=0, course=0;
bool fresh(bool have,uint32_t then) { return have && uint32_t(hal::millis()-then)<2000; }
int hex(char c) { return c>='0'&&c<='9'?c-'0':c>='A'&&c<='F'?c-'A'+10:c>='a'&&c<='f'?c-'a'+10:-1; }
bool number(const char* s,double& v) {
    if(!*s) return false;
    char* end=nullptr; v=std::strtod(s,&end);
    return *end==0 && std::isfinite(v);
}
bool coordinate(const char* s,const char* hem,bool lat,double& v) {
    double raw;
    if(!number(s,raw)||raw<0||std::strlen(hem)!=1) return false;
    const double deg=std::floor(raw/100), minutes=raw-deg*100;
    v=deg+minutes/60;
    if(minutes>=60||v>(lat?90:180)) return false;
    if(*hem==(lat?'S':'W')) v=-v;
    else if(*hem!=(lat?'N':'E')) return false;
    return true;
}
void time_field(const char* t) {
    if(std::strlen(t)<6) return;
    for(unsigned i=0;i<6;++i) if(t[i]<'0'||t[i]>'9') return;
    utc[0]=t[0]; utc[1]=t[1]; utc[2]=':'; utc[3]=t[2]; utc[4]=t[3];
    utc[5]=':'; utc[6]=t[4]; utc[7]=t[5]; utc[8]=0;
}
uint8_t parse() {
    if(length<9||line[length-3]!='*') return EV_NONE;
    const int a=hex(line[length-2]), b=hex(line[length-1]);
    if(a<0||b<0) return EV_NONE;
    uint8_t sum=0;
    for(size_t i=1;i<length-3;++i) sum^=uint8_t(line[i]);
    if(sum!=(a*16+b)) return EV_NONE;
    have_message=true; message_ms=hal::millis(); std::strcpy(last_line,line);
    line[length-3]=0;
    char* f[20]; unsigned n=1; f[0]=line;
    for(char* p=line;*p&&n<20;++p) if(*p==',') { *p=0; f[n++]=p+1; }
    if(std::strlen(f[0])!=6) return EV_NONE;
    if(!std::strcmp(f[0]+3,"RMC")) {
        have_speed=false;
        if(n<9||std::strcmp(f[2],"A")) { have_fix=false; return EV_NONE; }
        double v,h;
        if(!number(f[7],v)||v<0||v>600||!number(f[8],h)||h<0||h>360) return EV_NONE;
        speed=float(v*1.852); course=float(h); speed_ms=hal::millis(); have_speed=true;
        time_field(f[1]);
    } else if(!std::strcmp(f[0]+3,"GGA")) {
        have_fix=false;
        if(n<11) return EV_NONE;
        double q,sat,alt,lat,lon;
        if(!number(f[6],q)||!number(f[7],sat)) return EV_GGA;
        if(!(q==1||q==2||q==4||q==5)||sat<4||sat>99||
           !coordinate(f[2],f[3],true,lat)||!coordinate(f[4],f[5],false,lon)||
           !number(f[9],alt)||alt<-1000||alt>50000||std::strcmp(f[10],"M")) return EV_GGA;
        quality=int(q); satellites=int(sat); latitude=lat; longitude=lon; altitude=float(alt);
        if(std::strncmp(fix_utc,f[1],sizeof(fix_utc)-1)) {
            ++fix_seq; std::strncpy(fix_utc,f[1],sizeof(fix_utc)-1); fix_utc[sizeof(fix_utc)-1]=0;
        }
        time_field(f[1]); fix_ms=hal::millis(); have_fix=true;
        return EV_GGA|EV_FIX;
    }
    return EV_NONE;
}
void capture(uint8_t c) { ++bytes; if(boot_len<sizeof(boot)) boot[boot_len++]=c; }
}
void begin(uint32_t baud) {
    baud_now=baud; baud_index=0;
    for(unsigned i=0;i<sizeof(kBauds)/sizeof(kBauds[0]);++i) if(kBauds[i]==baud) baud_index=i;
    baud_since=hal::millis(); have_message=have_fix=have_speed=false;
    length=0; collecting=false; bytes=boot_len=0; utc[0]=last_line[0]=0; fix_seq=0; fix_utc[0]=0;
    hal::uart_config(kPort,baud);
}
void drain_rx() {
    uint8_t buf[128]; const size_t n=hal::uart_read(kPort,buf,sizeof(buf));
    for(size_t i=0;i<n;++i) capture(buf[i]);
}
uint8_t poll() {
    uint8_t events=EV_NONE,buf[128];
    for(unsigned batch=0;batch<4;++batch) {
        const size_t n=hal::uart_read(kPort,buf,sizeof(buf)); if(!n) break;
        for(size_t i=0;i<n;++i) {
            const char c=char(buf[i]); capture(buf[i]);
            if(c=='$') { collecting=true; length=0; }
            if(!collecting||c=='\r') continue;
            if(c=='\n') { line[length]=0; events|=parse(); length=0; collecting=false; }
            else if(length<sizeof(line)-1) line[length++]=c;
            else { collecting=false; length=0; }
        }
    }
    const uint32_t now=hal::millis();
    // Passive detection: do not change the receiver baud behind the host.
    if((!have_message||uint32_t(now-message_ms)>=5000)&&uint32_t(now-baud_since)>=2500) {
        baud_index=(baud_index+1)%(sizeof(kBauds)/sizeof(kBauds[0])); baud_now=kBauds[baud_index];
        hal::uart_config(kPort,baud_now); baud_since=now; have_message=have_fix=have_speed=false;
        length=0; collecting=false;
    }
    return events;
}
bool locked() { return fresh(have_fix,fix_ms); }
bool speed_valid() { return locked()&&fresh(have_speed,speed_ms); }
int sats() { return locked()?satellites:0; }
int fix() { return locked()?quality:0; }
const char* time_str() { return utc; }
float speed_kmh() { return speed_valid()?speed:0; }
float course_deg() { return speed_valid()?course:0; }
double lat_deg() { return locked()?latitude:0; }
double lon_deg() { return locked()?longitude:0; }
float alt_m() { return locked()?altitude:0; }
uint32_t rx_bytes() { return bytes; }
uint32_t fix_sequence() { return fix_seq; }
uint32_t current_baud() { return baud_now; }
bool nmea_valid() { return fresh(have_message,message_ms); }
const char* last_nmea() { return last_line; }
uint16_t boot_capture(const uint8_t*& data) { data=boot; return boot_len; }
}
