# Motor-enabled flight deployment — 2026-10-09

At the user's explicit request, the Raspberry Pi-connected controller was
programmed with **`v2_flight`**, not the motor-inhibited default profile.
The updated source GUI was launched on the Pi, closed for USB programming,
and reopened against `/dev/ttyACM0` afterward.

## Image and verification

- Firmware source: `23cc42ea4b7d89091da8ed466b934db896f1a2a9`.
- Profile: `v2_flight`, inheriting `FC_FLIGHT_ENABLED=1` from `v2_motor_test`.
- Board USB/ROM serial: `3154365B3034`.
- Rebuilt the flight profile successfully; entered ROM through the existing
  guarded USB DFU command.
- Saved the previous full 1,048,576-byte flash locally before programming.
- Programmed the 146,448-byte payload and verified every byte by DFU readback.
- Image including its 16-byte DFU suffix: 146,464 bytes;
  SHA-256 `2e17a145a0f77e6b50f495d6907649d15e8e0ebbd43a7d57298915201f45609d`.
- Payload/readback SHA-256:
  `d1ea0cc8b1258efb6808c292dabff3868698f3a53299d33d6de351199b3a46d9`.
- Returned to the application and confirmed live `flight_enabled=1` plus the
  new `YAW_STATUS` telemetry. Normal arming and failsafe rules remain active.

Backups, programmer logs, manifest and the short post-flash USB capture are
kept locally under `build/deploy-23cc42e-20261009/`; they are excluded from Git.

## Observed runtime status

Both BMI270s were healthy, selected-IMU gyro calibration completed, BMP581
was healthy, and no control timing fault was reported. The controller was
disarmed with RC failsafe active during the read. No motor/flight test was
performed.

BMM350 identified and initialized, produced 19 samples, then latched a
sample-read error: `healthy=0,stage=9,result=-2,bus_errors=1,last_reg=49`.
The previous firmware had the same error before programming (after four
samples). This is an observed software/driver status, not a diagnosis of
the PCB or its cause.

No installation calibration has been measured or installed. As intended,
the new firmware reports `source=GYRO,valid=0,reason=setup_required,
configured=0,hold=0`. Magnetic heading hold is unavailable until acquisition
is reliable and measured calibration passes the documented checks. See
[magnetic heading setup](MAGNETIC_HEADING.md).
