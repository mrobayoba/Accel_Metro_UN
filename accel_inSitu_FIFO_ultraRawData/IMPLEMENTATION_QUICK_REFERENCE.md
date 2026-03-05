# Quick Reference: Finding Changes in main.c

## File Locations of Key Changes

### 1. **Macros & Configuration (Lines 45-88)**
```
Section: /* USER CODE BEGIN PD */
Content: UART_DEBUG_ENABLE flag + LOG_CHUNK_SIZE, timing constants
```

### 2. **Global Variables (Lines 155-176)**
```
Section: /* USER CODE BEGIN PV */
After: char myFileName[15] = "record_1.txt";
Content: log_buf_a/b, active_log_buf/len, flush_pending, bytes_since_sync, etc.
```

### 3. **Function Prototypes (Lines 190-197)**
```
Section: /* USER CODE BEGIN 0 */
Content: dbg_uart(), dbg_uartf(), iis_read_fifo_dma(), flush_chunk(), flush_pending_buffers()
```

### 4. **EXTI Callback Update (Lines 540-554)**
```
Function: HAL_GPIO_EXTI_Callback()
Change: GPIO_PIN_9 now sets flag_fifo_irq = SET; (instead of calling iis_FIFO_read directly)
```

### 5. **Main Recording Loop (Lines 425-477)**
```
Location: if (flag_recordData) {
Changes:
  - Gate FIFO read with flag_fifo_irq check
  - Replace direct f_write with buffering logic
  - Call flush_pending_buffers() when full
  - Add sync policy (byte + timer threshold)
```

### 6. **Stop Sequence (Lines 383-413)**
```
Before: f_close(&myFile);
After: flush_pending_buffers() → f_sync() → f_close()
Effect: Ensures all data written before file closed
```

### 7. **Debug Wrapper Functions (Lines 585-618)**
```
Functions: dbg_uart(), dbg_uartf()
Feature: Compile-time gated with #if UART_DEBUG_ENABLE
```

### 8. **Buffering Helper Functions (Lines 620-650)**
```
Functions: flush_chunk(), flush_pending_buffers()
Location: USER CODE 4
Effect: Centralized SD write logic with validation
```

### 9. **DMA Callbacks (Lines 810-820)**
```
Functions: HAL_SPI_RxCpltCallback(), HAL_SPI_ErrorCallback()
Purpose: Signal DMA completion to waiting code
```

### 10. **DMA FIFO Read Function (Lines 823-860)**
```
Function: iis_read_fifo_dma()
Purpose: Replace blocking SPI read with DMA + timeout
```

### 11. **Updated iis_FIFO_read() (Lines 726-738)**
```
Change: iis_read(0x78, ...) → iis_read_fifo_dma(0x78, ...)
Effect: Payload transfer now uses DMA
```

---

## Quick Verification Steps

### ✅ Check Item 1 (Remove EXTI blocking)
```bash
grep -n "flag_fifo_irq = SET" main.c
# Should find: Line ~553 in EXTI callback (no iis_FIFO_read in ISR)
```

### ✅ Check Item 2 (Buffering)
```bash
grep -n "flush_log_buf\|active_log_buf" main.c
# Should find: Globals, and usage in recording loop
```

### ✅ Check Item 3 (DMA)
```bash
grep -n "iis_read_fifo_dma\|HAL_SPI_Receive_DMA" main.c
# Should find: Function definition + call in iis_FIFO_read()
```

### ✅ Check Item 6 (Sync policy)
```bash
grep -n "bytes_since_sync\|SYNC_BYTES_THRESHOLD" main.c
# Should find: Macro, counter update, sync check
```

### ✅ Check UART Debug Flag
```bash
grep -n "UART_DEBUG_ENABLE\|dbg_uart" main.c
# Should find: Macro definition + wrapper functions
```

---

## Enabling Debug Output

### To Enable:
1. Open `Core/Src/main.c`
2. Find line ~47: `#define UART_DEBUG_ENABLE 0`
3. Change to: `#define UART_DEBUG_ENABLE 1`
4. Rebuild and flash
5. UART2 (115200 baud) will show debug messages

### To Disable:
1. Change `#define UART_DEBUG_ENABLE 1` back to `0`
2. Rebuild
3. No UART overhead, faster captures

---

## Expected Behavior After Implementation

### Normal Operation
- Button press starts recording
- FIFO watermark IRQ sets `flag_fifo_irq` (very fast, no blocking)
- Main loop reads FIFO when flag is set
- Lines buffered in 4 KB chunks
- When chunk full, swapped to flush buffer
- Flush happens asynchronously in main loop
- Every 256 KB or 30 min → `f_sync()`
- Button press again stops recording
- Final flush + sync before file close

### With Debug Enabled (`UART_DEBUG_ENABLE = 1`)
- Periodic "Periodic sync OK" messages every 256 KB
- Error messages on SD write failures
- "SPI FIFO DMA read failed" if DMA timeout
- "Final flush failed" if last write fails

### Error Conditions
- SD write error → **Error LED ON** + `dbg_uart("f_write error...")`
- Sync failure → **Error LED ON** + `dbg_uart("f_sync failed...")` + safe stop
- DMA timeout → **Error LED ON** + `dbg_uart("SPI FIFO DMA read failed...")` + skip sample

---

## File Statistics

**Total lines in main.c:** 887 (was 690)  
**Lines added:** ~197  
**Net complexity increase:** Moderate (buffering + DMA)  
**Runtime overhead:** None (DMA offloads CPU)  
**Debug overhead:** Zero when `UART_DEBUG_ENABLE = 0`

---

## Compilation

```bash
# If building in STM32CubeIDE or similar:
make clean
make

# Expected result: 
# - No compilation errors
# - Some IntelliSense warnings (path config issue, not code)
# - Executable size ~40-50% larger (debug wrappers, double buffer)
```

---

## Rollback (if needed)

All changes are in `Core/Src/main.c` only. To revert:

1. Restore from git or backup
2. No config files or other sources modified
3. No CubeMX regeneration required (DMA already configured)

---

## Support / Debugging

If issues arise, check:

1. **FIFO not triggering:** Verify GPIO_PIN_9 interrupt is enabled in CubeMX
2. **SD write errors:** Check device capacity, formatting (FAT32), speed class
3. **DMA timeout (20ms):** Increase `SPI_DMA_TIMEOUT_MS` in macros if hardware is slow
4. **Dropped lines warning:** Reduce `LOG_CHUNK_SIZE` if buffer swap is too slow for your SD card
