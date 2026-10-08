# Headless parser checks: no serial port, Tk window or attached board required.
import ast
from pathlib import Path
import math, time
src=Path(__file__).resolve().parents[1]/'tools/gui.py'
tree=ast.parse(src.read_text(encoding='utf-8'))
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MonitorApp')
ns={'math':math,'time':time}
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(src),'exec'),ns)
class Value:
    value=''
    def set(self,value):self.value=value
    def configure(self,**kw):self.value=kw
app=ns['MonitorApp'].__new__(ns['MonitorApp'])
for key in ('roll','pitch','yaw','acc','gyr','pressure','temp','alt','fix','pos','gps_alt','gps_speed','gps_link','status','sats','gps_time','cal','imu_diag','imu1_diag','imu2_diag','bmp_health','sd'):
    setattr(app,key+'_var',Value())
app.status_lbl=Value();app.green='green';app.red='red';app.amber='amber'
app._last_att=app._last_bmp=app._last_gps=0
app._draw_horizon=lambda:None
app._handle_line('BMI_STATUS,1')
assert 'initialization OK' in app.status_var.value
app._handle_line('ATT,10,5,90')
assert app._last_att and '+10.0' in app.roll_var.value
app._handle_line('IMU_HEALTH,1,1,0,1,0')
assert app._last_att==0 and 'STALE' in app.roll_var.value and 'DISAGREE' in app.status_var.value
app._handle_line('BMP,1013.25,20,100')
assert app._last_bmp and '1013.250' in app.pressure_var.value
app._handle_line('GPS,40,29,100,9,1,12:00:00,30')
assert app._last_gps and '40.000000' in app.pos_var.value
app._expire_sensor_data(time.time()+3)
assert 'STALE' in app.pressure_var.value and 'STALE' in app.fix_var.value and app.pos_var.value=='--'
app._handle_line('ATT,nan,0,0')
assert app._last_att==0
app._handle_line('IMU_CONFIG,model=BMI270,mag=0,error0=0,error1=0')
assert app.imu_model=='BMI270'
app._handle_line('IMU_HEALTH,1,1,0,0,1')
assert 'BMI270 · #1 OK · #2 OK' in app.status_var.value
assert 'Errors 0 / 0' in app.imu_diag_var.value
app._handle_line('BMP_HEALTH,valid=1,error=0')
assert 'healthy' in app.bmp_health_var.value
app._handle_line('SD_DBG,0,,bytes=0,log_drops=0,usb_drops=12')
assert 'not logging' in app.sd_var.value and 'USB drops 12' in app.sd_var.value
app._handle_line('GPS_HEALTH,rx=0,nmea=0,fix=0,used=-1,visible=-1,baud=38400,bytes=0,messages=0')
assert 'No GNSS data' in app.gps_link_var.value and app.pos_var.value=='--'
app._handle_line('GPS_HEALTH,rx=1,nmea=0,fix=0,used=-1,visible=-1,baud=38400,bytes=14,messages=0')
assert 'no valid NMEA' in app.gps_link_var.value
app._handle_line('GPS_HEALTH,rx=1,nmea=1,fix=0,used=0,visible=0,baud=9600,bytes=500,messages=10')
app._expire_sensor_data(time.time())
assert 'Communication OK' in app.gps_link_var.value and 'no satellites' in app.fix_var.value and app.sats_var.value=='0'
app._handle_line('GPS_HEALTH,rx=1,nmea=1,fix=0,used=3,visible=1,baud=9600')
assert 'satellites in view' in app.fix_var.value and app.sats_var.value=='3'
app._handle_line('GPS_HEALTH,rx=1,nmea=1,fix=0,used=0,visible=-1,baud=9600')
assert 'visibility unknown' in app.fix_var.value
app._handle_line('GPS,40,29,100,9,1,12:00:00,30')
app._handle_line('GPS_HEALTH,rx=1,nmea=1,fix=1,used=9,visible=1,baud=9600')
assert app.pos_var.value!='--' and 'No fix' not in app.fix_var.value
app._handle_line('GPS_HEALTH,rx=1,nmea=1,fix=0,used=0,visible=1,baud=9600')
assert app.pos_var.value=='--' and 'No fix' in app.fix_var.value
before=app._gps_health.copy()
app._handle_line('GPS_HEALTH,rx=1,nmea=bad,fix=0,used=0,visible=0,baud=9600')
assert app._gps_health==before
app._expire_sensor_data(time.time()+4)
assert 'stale' in app.gps_link_var.value and app.sats_var.value=='--'
app._handle_line('IMU_CONFIG,model=BMI270,mag=0,error0=0,error1=103,regs0=240100,regs1=0')
assert 'No driver error' in app.imu1_diag_var.value and 'Chip ID mismatch' in app.imu2_diag_var.value
app._handle_line('IMU_HEALTH,1,0,0,0,1')
assert '#2 unavailable' in app.status_var.value and app.status_lbl.value['fg']=='amber'
app._handle_line('BMP_HEALTH,valid=0,error=2')
assert 'communication not established' in app.bmp_health_var.value
app._handle_line('BMP_HEALTH,valid=0,error=6')
assert 'Sample read failed' in app.bmp_health_var.value
before=app.status_var.value
app._handle_line('IMU_HEALTH,1,0,bad,0,1')
assert app.status_var.value==before
app._expire_sensor_data(time.time()+4)
assert 'stale' in app.status_var.value and 'stale' in app.imu2_diag_var.value and 'stale' in app.bmp_health_var.value
app._dfu_mode=True
app.status_var.set('ROM DFU active; flight application stopped')
app._expire_sensor_data(time.time()+5)
assert 'ROM DFU active' in app.status_var.value
print('28 GUI parser/freshness checks passed (headless; visual layout not tested)')
