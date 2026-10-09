# Asynchronous flight logging

The runtime logging path uses a 32 KiB RAM ring, preallocated contiguous FAT32 file, single-sector SDIO DMA writes, and a bounded card-command state machine. It has host fault tests and an STM32 build; real-card electrical/timing/power-loss qualification remains outstanding.

## Startup and runtime

Before the scheduler and motor authorization can run, `storage::prepare()` mounts the SD card and creates the next unused `FLT00000.BIN` ... `FLT09999.BIN` with `FA_CREATE_NEW`. Existing files are never overwritten. FatFs `f_expand(...,1)` allocates **128 MiB contiguously**; closing the file commits its full size and allocation before raw writes start. This allocation can take time at startup. FAT32 with 512-byte sectors is required. Insufficient contiguous space, wrong format, RNG failure or card initialization failure disables logging; it does not prevent MANUAL operation. No automatic formatting or deletion occurs.

During runtime, no FatFs calls, HAL command wait loops, busy waits, heap allocations, file allocation, file flushes or directory updates occur in the logger. A 4 kHz service task advances one bounded step: issue CMD24, poll its response, start DMA, poll transfer completion, issue/poll CMD13 until the card reports ready. DMA2 stream 6 / channel 4 belongs exclusively to the SD logger; SDIO uses PC8-PC12 and PD2. Do not share that stream or access the filesystem from other code while raw logging owns it.

The DMA source is an aligned static SRAM sector, not CCM or a stack buffer. The ring is consumed only after successful transfer AND card-programming completion. Command timeout is 5 ms; the entire sector operation is limited to 1 second. Failure disables the transfer path and latches logging off until restart; it never retries mounting or blocks control. No in-flight filesystem close/truncate is attempted. A full file stops logging without touching adjacent sectors.

`FC_SD_LOGGING=1` is the default. `FC_SD_LOGGING=0` disables the logger. Motor inhibition remains the separate default `FC_FLIGHT_ENABLED=0` gate. Enabling/building logging does not establish card reliability or flight qualification.

## Records and capacity

| Stream | Rate | Contents |
|---|---:|---|
| Each BMI270 | 400 Hz | Per-device sequence, reconstructed host sample time, batch sensor clock, pre-software-filter gyro/accel in body axes, filtered/calibrated values, calibration flag |
| Controller | 100 Hz | Roll/pitch rate demands, measured body rates, applied surface commands, throttle, mode, armed/failsafe/integrator flags |
| Flight state | 50 Hz | Existing 86-byte record: attitude, selected IMU, pressure/altitude/GPS, RC inputs and PWM commands |

IMU records are 42 bytes, controller records 28 bytes. Default payload is approximately **40,700 B/s**, plus sector envelopes. The 32 KiB ring covers about **0.8 seconds** at that rate; actual coverage varies with stream validity. Longer stalls drop complete new records and increment `LogRing::drops()` while preserving queued data. The file holds approximately **53 minutes** of continuous default data. These are capacity calculations, not measured card throughput.

Host IMU timestamps are reconstructed with 2.5 ms FIFO spacing, anchored at batch retrieval. They are not exact synchronized acquisition timestamps. The batch sensor clock and per-device sequence help identify timing gaps. The so-called raw samples already include the BMI270's sensor-side filtering and body-axis/sign transformation, but precede stored accelerometer calibration, gyro bias removal, the optional notch and software low-pass filters. A 400 Hz stream cannot characterize vibration above its 200 Hz Nyquist limit; sensor anti-alias filtering and higher-rate bench capture may be required.

## File recovery

Every 512-byte sector contains `FCS2`, a random 64-bit boot-session identifier, monotonically increasing sector sequence, payload length, up to 492 bytes of stream data, and CRC16/CCITT over the preceding 510 bytes. The first sector carries the legacy `STFC` stream header. Record types have their own magic and CRC.

Preallocated tail space is undefined and may contain old data. The decoder accepts only the current session's consecutive CRC-valid prefix. It stops at an unwritten, torn, mismatched or corrupt sector instead of combining stale tails with a new flight. A power loss can still lose queued data, the active sector, or data cached internally by a card. CRC/session checks detect many incomplete writes; they cannot guarantee consumer-card durability or recover a damaged first sector. The card's apparent file size stays 128 MiB.

```powershell
python tools/parse_bin_log.py FLT00000.BIN --summary
python tools/parse_bin_log.py FLT00000.BIN -o flight.csv
python tools/export_diagnostics.py FLT00000.BIN --out flight
python tools/analyze_imu.py flight_imu0.csv --out spectrum0.csv
```

The state decoder retains compatibility with older logs. Diagnostic export writes `flight_imu0.csv`, `flight_imu1.csv`, `flight_control.csv`, and a recovery-status JSON using bounded memory. Spectrum analysis requires NumPy and a contiguous sequence; it rejects missing samples rather than silently interpreting gaps as vibration. Analyze stationary and motor-running segments separately. The tools do not automatically change gains or filters.

## Acceptance work

With motors mechanically disabled, measure control-task maximum execution gaps while logging with several cards, near-full/fragmented cards, deliberate card removal, and long programming stalls. Verify DMA conflicts, CRCs, recovery after power cuts and meaningful per-IMU timing. Confirm no SD fault triggers a control deadline or unexpected reset. This physical validation has not been performed here.

The preallocation/raw extent calculation follows [FatFs f_expand documentation](https://elm-chan.org/fsw/ff/doc/expand.html). The register sequence follows the [ST STM32F4 SD driver](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_hal_sd.c) and RM0090, with command waiting replaced by explicit later-call polling. The implementation is deliberately tied to this F407/FatFs/FAT32 layout.

## Card status on 2026-10-09

The connected card (61,132,800 sectors of 512 bytes) mounted as FAT32, but boot preallocation failed with `FR_DENIED` (7). A user-requested, one-time on-controller FAT32 format succeeded and cleared that allocation failure. The temporary formatting command, reset request, and formatting implementation were then removed; `FF_USE_MKFS` is disabled again. Normal firmware never formats the card automatically.

**Runtime logging remains unresolved.** The final observed full `v2_flight` image creates `FLT00003.BIN`, then stops during the first DMA write with zero committed bytes. The captured failure is:

```text
SD_DBG,0,FLT00003.BIN,bytes=0,log_drops=0,usb_drops=0,stage=write_failed,fs=3,fatfs=0,hw=1052736,sectors=61132800,cmd=24,r1=2304,dma=65536,remaining=65471,dctrl=153
```

CMD24 returned `R1=0x900`; DMA2 stream 6 latched its FIFO error flag (`0x10000`). SDIO status, DMA status/count, and data-control registers are captured before stopping the transfer. This is evidence of the failure location, not a confirmed root cause. The DMA configuration now uses peripheral flow control and the transfer setup order from ST's [F407 board support implementation](https://github.com/STMicroelectronics/stm324xg-eval-bsp/blob/main/stm324xg_eval_sd.c) and HAL, but this did **not** resolve the observed write failure. Further SD investigation was deferred at the user's request.

`SD_DBG` now retains startup/failure stage, FatFs result, filesystem type, card sector count, and write-failure registers. The GUI distinguishes mount, format, allocation, file and write failures instead of displaying only “Inactive”; older firmware remains supported. Here `fs=3` is FatFs's FAT32 type, and `hw` is the HAL error at startup or the saved SDIO status after a runtime write failure. Diagnostic counters do not prove recovered log contents or flight qualification.

The flashed 151,020-byte application payload was read back byte-for-byte, SHA-256 `0d775d97e4fac2020d241fdae71fe102b45ff8a40716091b091f1ea469d4c964`. Motor-enabled flight configuration was preserved. Local deployment evidence is in ignored `build/deploy-sd-format-20261009/`; binaries and temporary maintenance utilities are not part of the repository.
