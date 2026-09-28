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
for key in ('roll','pitch','yaw','acc','gyr','pressure','temp','alt','fix','pos','gps_alt','gps_speed','status','sats','gps_time','cal'):
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
print('7 GUI parser/freshness checks passed (headless; visual layout not tested)')
