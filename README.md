# Accel Metro/UN — Vibration Logger Firmware

> Stable firmware for the in-situ vibration measurement system developed by Universidad Nacional de Colombia for the Metro de Medellín research project.

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware](#hardware)
3. [Pin Map](#pin-map)
4. [Firmware Architecture](#firmware-architecture)
5. [Step-by-Step Routine Descriptions](#step-by-step-routine-descriptions)
   - [Startup & Initialization](#1-startup--initialization)
   - [Sensor Initialization — `iis_init()`](#2-sensor-initialization--iis_init)
   - [Start Recording Flow](#3-start-recording-flow)
   - [Main Recording Loop](#4-main-recording-loop)
   - [FIFO Read — `iis_FIFO_read()`](#5-fifo-read--iis_fifo_read)
   - [Binary Packing into `bigbuf`](#6-binary-packing-into-bigbuf)
   - [SD Write — Sector-Aligned Flush](#7-sd-write--sector-aligned-flush)
   - [Periodic Sync & File Rotation](#8-periodic-sync--file-rotation)
   - [Stop Recording Flow](#9-stop-recording-flow)
   - [Interrupt Callbacks](#10-interrupt-callbacks)
   - [Timer Configuration](#11-timer-configuration)
6. [Binary File Format](#binary-file-format)
7. [LED Indicators](#led-indicators)
8. [Developer / Debug Mode](#developer--debug-mode)
9. [How to Use the Device](#how-to-use-the-device)
10. [Building & Flashing](#building--flashing)
11. [Post-Processing the Data](#post-processing-the-data)
12. [Key Configuration Constants](#key-configuration-constants)

---

## Overview

This firmware turns an STM32F411RE Nucleo board into a **high-speed 3-axis vibration logger**. It reads acceleration data at **~26,667 Hz** from an IIS3DWB sensor via SPI, buffers the samples in RAM, and writes them to an SD card in a compact binary format. Files are automatically rotated every 20 minutes so no single file becomes unmanageably large.

---

## Hardware

| Component | Part | Interface |
|---|---|---|
| Microcontroller | STM32F411RET6 (Nucleo-64) | — |
| Accelerometer | ST IIS3DWB | SPI1 |
| Storage | microSD / SD card | SDIO (4-bit) + DMA |
| Debug output | USB-Serial adapter | USART2 @ 115200 baud |
| User button | Nucleo onboard (PC13) | EXTI13 |

**System clock:** 100 MHz (HSI → PLL, PLLN=100, PLLM=8, PLLP=2)

---

## Pin Map

| STM32 Pin | Function | Direction |
|---|---|---|
| PA2 | USART2 TX (debug) | Output |
| PA3 | USART2 RX (debug) | Input |
| PA4 | Record LED (green) | Output |
| PA5 | SPI1 SCK | Output |
| PA6 | SPI1 MISO | Input |
| PA7 | SPI1 MOSI | Output |
| PA9 | IIS3DWB INT1 (FIFO WTM) | EXTI (rising) |
| PA11 | USART6 TX (unused alt) | — |
| PA12 | USART6 RX (unused alt) | — |
| PB4 | (Reserved / output OD) | Output |
| PB5 | (Reserved) | Output |
| PB6 | SPI1 CS (IIS3DWB) | Output (SW) |
| PC0 | Error LED (red) | Output |
| PC1 | Blink LED (status) | Output |
| PC8–PC12 | SDIO D0–D3, CK | SDIO AF |
| PC13 | User button (record toggle) | EXTI (rising) |
| PD2 | SDIO CMD | SDIO AF |

---

## Firmware Architecture

```
main()
 │
 ├─ Peripheral Init (HAL, GPIO, DMA, SDIO, FatFS, UART, TIM11, SPI1, TIM5)
 ├─ iis_init()        ← configure IIS3DWB sensor + FIFO
 ├─ Start TIM5 + TIM11 interrupts
 │
 └─ while(1) ──────────────────────────────────────────────────────────────
      │
      ├─ [flag_toggleRecord && flag_openFile]  → START RECORDING
      │     f_mount → open_new_file() → iis_write(ACCEL ON) → iis_write(FIFO ON)
      │
      ├─ [!flag_toggleRecord && flag_closeFile] → STOP RECORDING
      │     flush bigbuf → close_current_file() → f_unmount
      │
      └─ [flag_recordData]  → RECORDING LOOP
            1. iis_FIFO_read()       ← poll FIFO WTM, burst-read 140 bytes
            2. Pack samples into bigbuf (10 bytes/sample)
            3. UART preview (devMode, ~10 Hz)
            4. Flush bigbuf to SD when ≥512 bytes available
            5. f_sync every 5000 write-cycles
            6. f_sync on TIM5 interrupt (every 30 min)
            7. Rotate file every 20 min (open_new_file())

ISR callbacks (HAL):
  HAL_GPIO_EXTI_Callback(PC13)  → toggle flag_toggleRecord / flag_openFile
  HAL_GPIO_EXTI_Callback(PA9)   → iis_FIFO_read() (legacy, not used in main loop path)
  HAL_TIM_PeriodElapsedCallback(TIM5)  → set time_counter_done
  HAL_TIM_PeriodElapsedCallback(TIM11) → toggle Blink LED
```

---

## Step-by-Step Routine Descriptions

### 1. Startup & Initialization

`main()` executes the standard STM32 HAL boot sequence:

1. `HAL_Init()` — enables SysTick at 1 ms, sets HAL tick source.
2. `SystemClock_Config()` — configures the PLL to run the CPU at **100 MHz** from the internal HSI oscillator.
3. Peripheral initializers in order:
   - `MX_GPIO_Init()` — configures all GPIO pins (LEDs, button EXTI, SPI CS, SDIO, UART).
   - `MX_DMA_Init()` — enables DMA2 clock; DMA streams are linked to SPI1 and SDIO.
   - `MX_SDIO_SD_Init()` — sets up the SDIO peripheral in 1-bit mode initially; FatFS/BSP will widen to 4-bit.
   - `MX_FATFS_Init()` — links FatFS to the SD disk driver.
   - `MX_USART2_UART_Init()` — 115200-8-N-1, used for debug output.
   - `MX_TIM11_Init()` — status-blink timer (period ≈ 100 ms → ~5 Hz blink in idle).
   - `MX_SPI1_Init()` — SPI master, CPOL=0, CPHA=1 (Mode 1), prescaler /8 → ~12.5 MHz.
   - `MX_TIM5_Init()` — 30-minute periodic sync timer.
4. `iis_init()` — initializes the IIS3DWB accelerometer.
5. Both timers are started with interrupts enabled.

---

### 2. Sensor Initialization — `iis_init()`

This function configures the IIS3DWB over SPI to produce timestamped acceleration data in continuous FIFO mode:

| Step | Register | Value | Effect |
|---|---|---|---|
| 1 | `0x12` | `0x03` | Software reset (waits 10 ms) |
| 2 | `0x0F` | read | Read WHO_AM_I, printed to UART |
| 3 | `0x10` | `0x00` | CTRL1_XL: accelerometer OFF |
| 4 | `0x15` | `0x00` | CTRL6: all 3 axes enabled |
| 5 | `0x17` | `0x00` | CTRL8: bandwidth 6.3 kHz |
| 6 | `0x19` | `0x20` | CTRL10: enable internal timestamp counter |
| 7 | `0x0A` | `0x00` | FIFO_CTRL4: reset FIFO |
| 8 | `0x07/08` | WTM=20 | FIFO watermark threshold = 20 words |
| 9 | `0x09` | `0x0A` | FIFO_CTRL3: batch data rate = 26,667 Hz |
| 10 | `0x0A` | `0x46` | FIFO_CTRL4: continuous FIFO mode |

At this point the sensor FIFO is running but the accelerometer output data rate (ODR) is still zero — the accel is turned **on** only when recording actually starts (Step 3 below).

---

### 3. Start Recording Flow

Triggered by **pressing the user button (PC13)** once:

1. `HAL_GPIO_EXTI_Callback` sets `flag_toggleRecord = 1` and `flag_openFile = 1`.
2. In the main loop, the firmware detects both flags and:
   - Calls `f_mount()` to mount the FAT filesystem.
   - Reports total and free SD space via UART.
   - Calls `open_new_file()` to create `rec_A0.bin` with a text header.
   - Turns on the **Record LED** (PA4).
   - Writes to `0x10` (`0xA8`) to enable the accelerometer at 26,667 Hz, ±4g.
   - Resets and starts the FIFO: write `0x42`=`0xAA` (timestamp reset), `0x0A`=`0x40` (clear FIFO), `0x0A`=`0x46` (continuous mode).
   - Resets counters (`write_count`, `bigbuf_len`, `file_start_tick`).
   - Reads `freq_fine` (register `0x63`) to compute the actual ODR.
   - Sets `flag_recordData = 1` to enter the recording loop.

#### `open_new_file()`

Creates a file named `rec_A{N}.bin` (N = `file_index`, starting at 0) and writes a **plain-text header** before binary data begins:

```
BINARY FORMAT: 10 bytes per sample
Each sample: uint32_t timestamp(LSB), int16_t X, int16_t Y, int16_t Z
Timestamp: 1 LSB = 12.5us, Accel: 1 LSB = 0.122mg, +-4g
WTM=20 ODR~26667Hz file_index=0
DATA_START
```

Everything after `DATA_START\n` is raw binary.

---

### 4. Main Recording Loop

While `flag_recordData == 1`, the main loop runs the following pipeline on every iteration:

```
iis_FIFO_read → pack to bigbuf → [optional UART preview] → flush to SD → [f_sync] → [rotate file]
```

---

### 5. FIFO Read — `iis_FIFO_read()`

This is the core data-acquisition routine. It uses **polling** rather than interrupts for the main recording path:

1. **Poll FIFO status** — reads register `0x3B` (FIFO_STATUS2) repeatedly until bit 7 (`WTM_IA`) is set, meaning at least 20 words are available. Timeout after 1000 retries to avoid infinite blocking.
2. **Burst read** — reads `WTM_THRESHOLD × 7 = 140 bytes` from register `0x78` (FIFO_DATA_OUT) in a single SPI transaction.
3. **Parse tags** — each 7-byte FIFO word starts with a tag byte (bits [7:3]):
   - `tag == 0x02` → Accelerometer sample: bytes 1–6 are X, Y, Z in little-endian int16.
   - `tag == 0x04` → Timestamp: bytes 1–4 form a 32-bit timestamp counter (1 LSB = 12.5 µs).
4. **Pair matching** — an accel sample and a timestamp are paired together; once both are collected, they are stored in the output arrays at `out_idx` and the index advances. Up to 10 pairs (WTM/2) are returned per call.

> **Note:** The EXTI callback on PA9 (IIS3DWB INT1) also calls `iis_FIFO_read()` directly, which was the original interrupt-driven design. In the current polling-based main loop, this ISR call is redundant but harmless.

---

### 6. Binary Packing into `bigbuf`

After each `iis_FIFO_read()` call, the 10 sample pairs are serialized into the 50 KB accumulation buffer `bigbuf`:

```
For each valid sample (timestamp != 0):
  bigbuf[n+0..3] = timestamp (uint32, little-endian)
  bigbuf[n+4..5] = X (int16, little-endian)
  bigbuf[n+6..7] = Y (int16, little-endian)
  bigbuf[n+8..9] = Z (int16, little-endian)
  bigbuf_len += 10
```

Samples with `timestamp == 0` are silently skipped (incomplete FIFO word).

---

### 7. SD Write — Sector-Aligned Flush

To maximize SD write throughput, the firmware only flushes when it has accumulated at least one full 512-byte sector:

```c
to_write = (bigbuf_len / 512) * 512;   // round down to sector boundary
f_write(&myFile, bigbuf, to_write, &aux);
memmove(bigbuf, bigbuf + to_write, remainder); // keep leftover bytes
bigbuf_len = remainder;
write_count++;
```

If `f_write` returns an error or writes fewer bytes than requested, the **Error LED** (PC0) is lit and `ERR:SD_WR` is sent over UART.

The 50 KB buffer provides approximately **187 ms** of sensor data at 26,667 Hz before it must be flushed, giving the SD card plenty of time to complete DMA transfers.

---

### 8. Periodic Sync & File Rotation

Two independent mechanisms keep data safe on the SD card:

| Mechanism | Trigger | Action |
|---|---|---|
| **Count-based sync** | Every 5000 write cycles | `f_sync()` |
| **Timer-based sync** | TIM5 period elapsed (~30 min) | `f_sync()` via `time_counter_done` flag |
| **File rotation** | `HAL_GetTick() - file_start_tick >= 1,200,000 ms` | Flush remainder, `f_close()`, `file_index++`, `open_new_file()` |

File names after rotation: `rec_A0.bin`, `rec_A1.bin`, `rec_A2.bin`, …

---

### 9. Stop Recording Flow

Triggered by **pressing the user button (PC13) again**:

1. `HAL_GPIO_EXTI_Callback` sets `flag_toggleRecord = 0`.
2. In the main loop: `flag_closeFile` is detected, `flag_recordData = 0`.
3. Any remaining bytes in `bigbuf` are flushed with `f_write()`.
4. `close_current_file()` is called:
   - `f_close()` closes the active file.
   - IIS3DWB accelerometer is turned off (`0x10 = 0x08`).
   - FIFO is reset (`0x0A = 0x40`).
   - `f_mount(NULL, ...)` unmounts the filesystem.
5. **Record LED** turns off; the status **Blink LED** resumes.

---

### 10. Interrupt Callbacks

#### `HAL_GPIO_EXTI_Callback(GPIO_Pin)`

| Pin | Condition | Action |
|---|---|---|
| PC13 (button) | `flag_toggleRecord == 0` | Set `flag_toggleRecord = 1`, `flag_openFile = 1` (start) |
| PC13 (button) | `flag_toggleRecord == 1` | Clear `flag_toggleRecord` (stop) |
| PA9 (IIS3DWB INT1) | always | Call `iis_FIFO_read()` (legacy interrupt path) |

#### `HAL_TIM_PeriodElapsedCallback(htim)`

| Timer | Action |
|---|---|
| TIM5 | If recording: set `time_counter_done = 1` → triggers `f_sync` in main loop |
| TIM11 | Toggle Blink LED (heartbeat in idle/standby) |

---

### 11. Timer Configuration

| Timer | Prescaler | Period | Actual Period |
|---|---|---|---|
| **TIM5** (sync/heartbeat) | 50,000 − 1 | 2 × 1000 × 60 × 30 = 3,600,000 | ~30 minutes |
| **TIM11** (blink) | 50,000 − 1 | 2 × 100 = 200 | ~100 ms |

Both timers use the 100 MHz APB2 timer clock. With prescaler = 49999, the timer tick = 0.5 ms. TIM11 then counts 200 ticks → 100 ms period → LED blinks at ~5 Hz when idle.

---

## Binary File Format

Each `.bin` file starts with a plain-text header (variable length, ends with `DATA_START\n`). After that, samples are packed back-to-back with no separators:

```
Offset  Size  Type      Description
------  ----  --------  ------------------------------------
0       4     uint32_t  Timestamp counter (1 LSB = 12.5 µs)
4       2     int16_t   X-axis acceleration (1 LSB = 0.122 mg)
6       2     int16_t   Y-axis acceleration (1 LSB = 0.122 mg)
8       2     int16_t   Z-axis acceleration (1 LSB = 0.122 mg)
```

**Total: 10 bytes per sample. At 26,667 Hz, this is ~267 KB/s (≈16 MB/min).**

To convert raw values:
- **Time (ms):** `timestamp × 12.5 / 1000`
- **Acceleration (mg):** `raw × 0.122`
- **Acceleration (g):** `raw × 0.000122`
- **Acceleration (m/s²):** `raw × 0.001197`

---

## LED Indicators

| LED | Color | Pin | Meaning |
|---|---|---|---|
| Record LED | Green | PA4 | ON while recording |
| Error LED | Red | PC0 | ON on SD mount error, write error, or file open error |
| Blink LED | — | PC1 | Toggles at ~5 Hz when idle (standby heartbeat) |

---

## Developer / Debug Mode

The `devMode` variable in `main.c` controls UART output:

```c
uint8_t devMode = 0;  // 0 = silent (production), 1 = verbose (debug)
```

| `devMode` | Behavior |
|---|---|
| `0` | No UART output. All `send_uart2()` calls are no-ops. Use this on the train. |
| `1` | Prints SD info, recording start/stop timestamps, file rotation events, and a live data preview at ~10 Hz (`ts_ms, ax_mg, ay_mg, az_mg`). Connect at **115200 baud, 8-N-1** (e.g., CoolTerm, PuTTY). |

To change the mode: edit the variable in `main.c` and recompile.

---

## How to Use the Device

### Prerequisites

- Formatted SD card (FAT32, recommended ≤32 GB).
- Firmware already flashed to the STM32F411RE Nucleo board.
- (Optional) USB-serial adapter on PA2/PA3 for debug monitoring.

### Step-by-Step Operation

1. **Insert the SD card** into the SD slot before powering on the board.
2. **Power on** the board via USB or the barrel-jack power supply.
   - The **Blink LED** (PC1) will start flashing, indicating the device is in standby.
   - If `devMode = 1`, the message `Starting Accel Metro-UN` and `Ready to roll!!!` will appear on UART.
3. **Start recording** by pressing the **blue user button** (PC13) once.
   - The **Blink LED** stops.
   - The **Record LED** (PA4) turns on solid green.
   - The SD card is mounted and file `rec_A0.bin` is created.
   - The accelerometer begins sampling at ~26,667 Hz.
4. **Leave the device running.** Files rotate automatically every 20 minutes (`rec_A0.bin` → `rec_A1.bin` → …). No user action is required.
5. **Stop recording** by pressing the **blue user button** again.
   - The Record LED turns off.
   - All buffered data is flushed to the SD card.
   - The SD card is safely unmounted.
   - The Blink LED resumes.
6. **Remove the SD card** and transfer the `.bin` files to a PC for analysis.

> **Important:** Always stop recording (step 5) before removing power or the SD card. Failure to do so may corrupt the current file. Previously closed/rotated files are safe.

### Error Recovery

- **Error LED (PC0) ON at startup/mount:** The SD card was not detected or is not formatted as FAT32. Re-insert the card or reformat it and press the button again.
- **Error LED ON during recording:** An SD write error occurred. Stop recording, remove the SD, check it with `chkdsk` / `fsck`, and restart.
- **Device unresponsive:** Power-cycle the board. Check that the SD card is properly seated.

---

## Building & Flashing

This project targets **STM32CubeIDE** (or any GCC-ARM cross-compiler toolchain):

1. Open STM32CubeIDE and import the project folder `accel_inSitu_FIFO_ultraRawData/`.
2. Select the **Debug** or **Release** build configuration.
3. Build with `Project → Build Project` (or `Ctrl+B`).
4. Flash with `Run → Debug` (ST-Link on the Nucleo) or use the `.launch` file provided:
   `accel_inSitu_FIFO_ultraRawData Debug.launch`

Linker scripts provided:
- `STM32F411RETX_FLASH.ld` — normal execution from flash.
- `STM32F411RETX_RAM.ld` — execution from RAM (for fast iteration testing).

---

## Post-Processing the Data

Each `.bin` file has a plain-text header of variable size. To parse:

1. Open the file as binary.
2. Search for the byte sequence `DATA_START\n` (ASCII).
3. Read from the byte immediately after that sequence onward.
4. Parse 10-byte records as described in [Binary File Format](#binary-file-format).

**Example Python snippet:**

```python
import numpy as np
import struct

with open("rec_A0.bin", "rb") as f:
    raw = f.read()

marker = b"DATA_START\n"
start = raw.index(marker) + len(marker)
data = raw[start:]

n_samples = len(data) // 10
samples = np.frombuffer(data[:n_samples * 10], dtype=np.dtype([
    ("ts",  "<u4"),
    ("x",   "<i2"),
    ("y",   "<i2"),
    ("z",   "<i2"),
]))

time_ms  = samples["ts"] * 12.5 / 1000.0   # milliseconds
accel_x  = samples["x"]  * 0.122            # milli-g
accel_y  = samples["y"]  * 0.122            # milli-g
accel_z  = samples["z"]  * 0.122            # milli-g
```

---

## Key Configuration Constants

All tunable parameters are defined at the top of `main.c`:

| Constant | Default | Description |
|---|---|---|
| `WTM_THRESHOLD` | `20` | FIFO watermark — words read per polling cycle |
| `BIG_BUF_SIZE` | `50000` | RAM accumulation buffer size (bytes) |
| `WRITE_THRESHOLD` | `512` | Minimum bytes before flushing to SD (1 sector) |
| `SYNC_EVERY_N` | `5000` | Number of SD write cycles between `f_sync` calls |
| `FILE_ROTATE_MS` | `1200000` | File rotation interval (ms) — default 20 minutes |
| `PREVIEW_EVERY_N` | `130` | UART preview cadence (~10 Hz) when `devMode = 1` |
| `BUFFER_SIZE` | `72` | Auxiliary `sprintf` buffer size (bytes) |
 
