"""Explicitly simulated preview. Never connects to hardware or writes commands."""
import math


def position(t):
    return 47.3977 + .0020*math.sin(t/35), 8.5456 + .0038*math.cos(t/35)


def frame(t):
    roll = 19*math.sin(t/5); pitch = 6+5*math.sin(t/9); yaw = (t*1.8)%360
    lat, lon = position(t)
    rc = [1500+int(roll*7), 1500+int(pitch*8), 1320, 1496, 1000, 1500, 1500, 1000]+[1500]*8
    return [f"ATT,{roll:.2f},{pitch:.2f},{yaw:.2f}",
            "IMU_CONFIG,model=BMI270,mag=0,error0=0,error1=0,regs0=240100,regs1=240100",
            "IMU_HEALTH,1,1,0,0,1", "BMP_HEALTH,valid=1,error=0",
            "BMI,0.01,0.02,0.998,0.12,-0.05,0.18", "BMP,998.4,24.6,126.5",
            f"EST,bias_ready=1,gbias=0.01/-0.02/0.00,acc_trust=0.98,agl_m={42+4*math.sin(t/9):.1f},climb_mps={.45*math.cos(t/9):.2f},baro_ref=1",
            "MODE,active=ASSIST,req=ASSIST,armed=0,failsafe=0,assist_lockout=0,flight_enabled=0,timing_fault=0,flying=0",
            f"GPS,{lat:.6f},{lon:.6f},488.2,14,1,12:34:56,{58+3*math.sin(t/6):.1f}",
            "GPS_HEALTH,rx=1,nmea=1,fix=1,used=14,visible=1,baud=9600,bytes=125800,messages=1800",
            "CRSF_STAT,receiving=1,frames_ok=15432,crc_err=0,resync=0,telem_tx=904",
            "LINK,up_rssi_dbm=-67,up_lq=99,up_snr=9,rf_mode=5",
            "RC,"+",".join(map(str,rc)),
            f"OUT,{1500+int(roll*5)},1515,1000,1496,1500,{1500-int(roll*5)},1000,1000",
            "SD_DBG,1,FL000042.BIN,bytes=1843200,log_drops=0,usb_drops=0",
            "TELEM,300,300,50,50,0,0", "SCHED,control*,hz=400,last_us=54,max_us=108,overruns=0"]
