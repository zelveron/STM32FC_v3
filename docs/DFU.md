# USB firmware updates (STM32F407VG)

Use DFU on the bench with the aircraft landed, propulsion disconnected, CH5 off and throttle low. Entering ROM stops the flight application, including stabilization, telemetry and PWM. This is a maintenance operation, not a flight mode or a transmitter switch assignment.

## Dashboard

Run the [standalone Windows GUI](GUI_WINDOWS.md) or `python tools/gui.py` (optionally add `--port COM6`). Click **Enter DFU**. The dashboard stops its reader, closes the serial handle, sends the command and checks for **0483:df11** using dfu-util. It reports **DFU VERIFIED** only after the internal-flash interface appears. Monitoring stays paused so the programmer can use USB. After programming and restarting the board, click **Reconnect**. If Windows assigned a different COM number, choose **Auto-detect STM32**, or use **Refresh ports** and select the new COM port before reconnecting. The Windows EXE includes dfu-util; source-based runs still need it installed.

## Command line

Install the tools with `python -m pip install pyserial platformio`, then build the matching profile. `enter_dfu.py` finds dfu-util on PATH or in PlatformIO's `tool-dfuutil/bin`; use `--dfu-util PATH` for another installation.

Close any dashboard/serial monitor first:

```text
python tools/enter_dfu.py --port COM6
```

On Linux use `/dev/ttyACM0` instead of `COM6`. Omitting `--port` detects exactly one STM32 CDC device. The helper **only enters and verifies DFU**; it does not erase or program anything. A refused command, missing DFU driver, disappearance without DFU, or multiple DFU devices is an error, not success.

The wire command is the exact ASCII byte sequence `dfu\n`; the helper sends `\ndfu\n` to discard an incomplete preceding command. `REBOOT_BL\n` remains an alias. CRLF is accepted; binary/NUL and overlong lines are rejected. An acknowledgment may be lost as USB resets, so it is not used as proof of DFU.

## Firmware gates

- Reject if armed or the flight integration state remains active.
- With fresh RC input, require CH5 at/below its arming threshold and CH3 at/below 1050 microseconds.
- After any arming during that boot, require a fresh RC link and completed failsafe recovery as well as disarm/idle. An RC-loss disarm cannot authorize DFU.
- Before the first arming, an RC receiver is not required for USB maintenance.

These checks cannot prove that an aircraft has landed. The operator must only request DFU on the ground. Refusal is `NAK,DFU,DISARM_IDLE_REQUIRED`. No command is sent over CRSF, and the transmitter mapping is unchanged.

## Select, program and verify

The current Raspberry Pi-connected card was identified by the user on 2026-10-08 as **BMI270**. Use the default `v2_bmi270` profile, which preserves disabled magnetometer support and motor inhibition. BMI270 is the only supported IMU configuration.

```text
pio run -e v2_bmi270
dfu-util -l
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D .pio/build/v2_bmi270/firmware.bin
```

The installed PlatformIO dfu-util may need its full executable path on Windows. For multiple devices, select the intended ROM serial with `-S SERIAL`. This board's ROM serial is `3154365B3034`; normal v3 CDC uses the same serial and currently enumerates as `/dev/ttyACM0` on the Raspberry Pi. Windows assigns its own COM number.

For a full backup **before an update**, use a new output filename:

```text
dfu-util -d 0483:df11 -S 3154365B3034 -a 0 -s 0x08000000:1048576 -U original-flash.bin
```

For byte-for-byte verification, download without `:leave`, upload the programmed payload length to another file, compare it to the image payload, then leave:

```text
dfu-util -d 0483:df11 -S 3154365B3034 -a 0 -s 0x08000000:leave
```

PlatformIO adds a **16-byte DFU suffix** to its `.bin`; dfu-util strips this metadata during programming. Compare the readback with the binary excluding that suffix (verify the `UFD` suffix signature first). Do not write the suffix as program data. Do not use mass-erase, unprotect or option-byte changes for this normal update procedure.

Allow several seconds for CDC to return. If it does not, press RESET with BOOT released. If the application is unresponsive, software DFU cannot run; BOOT/RESET or SWD remains the recovery path.

## Reset implementation and evidence

`src/hal/stm32/bootloader.cpp` stores a magic value plus its complement, clears old reset flags, and issues a system reset. `ld/stm32f407vg.ld` reserves eight NOLOAD bytes at `0x2001FFF8` and moves the stack top below that mailbox. The early `.preinit_array` hook consumes the marker before Arduino initializes clocks, USB, timers or the watchdog. Only a matching marker plus a software reset is accepted; power, brownout and watchdog resets boot the application. ROM vectors at `0x1FFF0000` are remapped, then MSP and the reset entry are loaded together in assembly.

The early reset path avoids jumping into ROM with the application watchdog already running. **CDC → ROM DFU → CDC and programmed-payload readback were verified on the current BMI270 card on 2026-10-08.** The original 1 MiB flash was backed up before updating. See [assembled-board results](ASSEMBLED_BOARD.md) for sensor/runtime observations.

References: [ST AN2606 system-memory bootloader](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf), [dfu-util manual](https://dfu-util.sourceforge.net/dfu-util.1.html). The supplied FlightController 0.3.1 project's command and reset-marker approach were inspected as a reference; its build scripts and binaries were not executed.
