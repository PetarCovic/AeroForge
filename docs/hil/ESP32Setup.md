# ESP32 Hardware-in-the-Loop Setup

## Verified board and connection

The board used for the recorded AeroForge hardware run is an **ESP32-D0WD-V3 (revision 3.1)** with a **CH340 USB-to-serial adapter**. Windows exposed it as `USB-SERIAL CH340 (COM3)` during the recorded tests. `COM1` was the PC's legacy communications port, not the ESP32.

The ESP32-D0WD-V3 belongs to the original **ESP32** chip family. It is not an ESP32-S3, even if an older firmware directory still contains `esp32s3` in its path. The ESP-IDF target must be `esp32`.

## Hardware requirements

- ESP32 development board with a working CH340 USB-UART connection.
- USB **data** cable and available Windows USB port.
- ESP-IDF installation and an ESP-IDF-enabled PowerShell session.
- AeroForge Windows x64 Release build containing the hardware benchmark.
- No SD card or external sensors are required for the initial simulated-sensor controller test.

The simulation uses virtual IMU inputs; do not attach physical motors, servos, rocket propulsion, or other actuators for these communication and safety checks.

## Identify the COM port

In PowerShell:

```powershell
Get-PnpDevice -Class Ports | Select-Object Status, FriendlyName, InstanceId
```

Look for an entry such as `USB-SERIAL CH340 (COM3)`. The actual COM number is assigned by Windows and may differ on another computer or after reconnecting the board.

From the ESP-IDF environment, independently identify the chip:

```powershell
esptool -p COM3 chip-id
```

The recorded identification was `ESP32-D0WD-V3`. A message saying `This chip is ESP32, not ESP32-S3` means the selected target is wrong; do not force an S3 image onto the original ESP32.

## Build and flash firmware

From the repository root in an ESP-IDF PowerShell session:

```powershell
cd firmware/esp32s3
idf.py set-target esp32
idf.py build
idf.py -p COM3 -b 115200 flash
```

The directory name is a legacy source-layout detail; selecting the **correct chip target** is mandatory. `idf.py set-target esp32` may regenerate build configuration and defaults. Preserve any intended application UART configuration when changing targets.

The `-b 115200` value in the **flash** command controls programming speed. It is independent of the application's runtime UART rate.

## Configure the application UART

The original ESP32 firmware uses UART0, normally GPIO1 for TX and GPIO3 for RX, with 8-N-1 framing. The successful initial test used a firmware application baud of **115200**. The later hardware benchmark used **460800** after firmware and host were configured to match.

If the firmware has a direct baud constant in `firmware/esp32s3/main/main.cpp`, set it to the same value that the PC application will open. If the project includes `main/Kconfig.projbuild` and uses `CONFIG_AEROFORGE_HIL_UART_BAUD`, the setting may instead be changed through `idf.py menuconfig`.

A menu called **AeroForge HIL Controller** only appears if that Kconfig file is present and recognized. Its absence does not mean the original ESP32 cannot run the firmware.

After changing the runtime baud, rebuild and reflash, then pass the matching value to AeroForge.

## Run a physical benchmark

Close `idf.py monitor` and other applications using COM3. From the AeroForge root:

```powershell
.\out\build\x64-release-clean\AeroForgeHilHardwareBenchmarks.exe COM3 1000 out\hil-benchmarks\hardware-460800 460800
```

This is a **100 Hz simulation benchmark with asynchronous controller responses**. The recorded result had 641 accepted controller responses across 1,000 physics steps, 997 steps with still-valid control, and zero measured simulation deadline misses. It did **not** meet a 90% newly accepted response threshold.

## Review saved results

```powershell
.\out\build\x64-release-clean\AeroForgeVerificationAudit.exe hardware out\hil-benchmarks\hardware-460800\hil_hardware_samples.csv out\hil-benchmarks\hardware-460800\hil_hardware_summary.json out\verification\HardwareAudit.md
```

The demonstrated evidence audit passed with a warning about response acceptance. That warning must not be suppressed or described as a passed 100 Hz controller response requirement.

## Troubleshooting

| Symptom | Check |
|---|---|
| Esptool connects to original ESP32 but build targets S3 | Run `idf.py set-target esp32` and rebuild |
| COM port cannot be opened | Close monitors and other applications using it |
| No valid responses | Check firmware application baud and host baud match |
| Serial text at startup | ESP32 ROM may print before application binary protocol begins; parser must resynchronize |
| Flash cannot connect | Verify correct COM port, data cable, USB driver, and BOOT/RESET entry procedure |
| 100 Hz simulation but fewer responses | Distinguish scheduled physics steps from fresh controller responses; inspect HIL performance report |
| GUI does not communicate but benchmark does | Check whether GUI is still configured for 115200 baud |

## Safety checks to sign off separately

With **no physical actuators attached**, test GUI start/stop, COM-port contention, USB removal, watchdog zero-moment fallback after command expiry, clean COM-port release, and fresh-session recovery. These manual checks were not established as complete by the reported benchmark or CSV/JSON audit.

## Related documents

- [HIL architecture](HIL_ARCHITECTURE.md)
- [Wire protocol](HilWireProtocol.md)
- [HIL measurements](../performance/HilPerformanceReport.md)
