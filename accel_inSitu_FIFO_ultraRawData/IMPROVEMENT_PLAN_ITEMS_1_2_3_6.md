# STM32 IIS3DWB + SD Logger Improvement Plan (Explicit Code Changes)

Scope: only item 1, item 2, item 3 and item 6 (without preallocation).

- Item 1: remove race conditions / heavy ISR work
- Item 2: double-buffered SD writes
- Item 3: SPI FIFO payload read over DMA
- Item 6: robust FatFs write+sync policy (no f_expand)
- Extra requested: wrap UART debug calls behind a handcoded flag

This file is the implementation plan with explicit code edits. No source code is changed yet.

---

## 0) Global code edits to apply first (includes, macros, globals, prototypes)

File: `Core/Src/main.c`

### 0.1 Add include
In USER CODE Includes section, add:

```c
#include <stdarg.h>
```

### 0.2 Add handcoded UART debug control flag and wrappers
In USER CODE PD section, add:

```c
#define UART_DEBUG_ENABLE 0   // Handcoded: set to 1 to enable UART logs, 0 to compile out debug logs

#define LOG_CHUNK_SIZE       4096U
#define LOG_LINE_MAX         64U
#define SYNC_BYTES_THRESHOLD (256U * 1024U)
#define SPI_DMA_TIMEOUT_MS   20U
```

In USER CODE 0 section, add prototypes:

```c
void dbg_uart(const char *msg);
void dbg_uartf(const char *fmt, ...);

HAL_StatusTypeDef iis_read_fifo_dma(uint8_t reg, uint8_t *dst, uint16_t len);
FRESULT flush_chunk(char *buf, uint32_t len);
FRESULT flush_pending_buffers(void);
```

### 0.3 Add globals for IRQ-safe flow, buffering, DMA sync, FatFs accounting
In USER CODE PV section, replace ISR-shared flags with volatile and add:

```c
volatile uint8_t flag_recordData = RESET;
volatile uint8_t flag_toggleRecord = RESET;
volatile uint8_t flag_openFile = RESET;
volatile uint8_t flag_closeFile = RESET;
volatile uint16_t time_counter_done = RESET;

volatile uint8_t flag_fifo_irq = RESET;

volatile uint8_t spi_fifo_dma_done = RESET;
volatile uint8_t spi_fifo_dma_error = RESET;

char log_buf_a[LOG_CHUNK_SIZE] = {0};
char log_buf_b[LOG_CHUNK_SIZE] = {0};
char *active_log_buf = log_buf_a;
char *flush_log_buf  = log_buf_b;
uint32_t active_log_len = 0U;
uint32_t flush_log_len = 0U;
volatile uint8_t flush_pending = RESET;

uint32_t bytes_since_sync = 0U;
uint32_t dropped_lines = 0U;
```

---

## 1) Item 1 — remove race conditions and heavy ISR work

File: `Core/Src/main.c`

### 1.1 EXTI callback: remove FIFO read call
Replace this block:

```c
if (GPIO_Pin == GPIO_PIN_9) { // Accel_pin
  iis_FIFO_read(datax, datay, dataz, time); // to read data from the FIFO
}
```

With:

```c
if (GPIO_Pin == GPIO_PIN_9) { // Accel FIFO watermark IRQ
  flag_fifo_irq = SET;
}
```

### 1.2 Main loop: only process FIFO when IRQ flag is set
Replace unconditional call in record loop:

```c
iis_FIFO_read(datax, datay, dataz, time);
```

With gate:

```c
if (flag_fifo_irq) {
  flag_fifo_irq = RESET;
  iis_FIFO_read(datax, datay, dataz, time);
}
```

### 1.3 Keep all parsing/storage in main context
No FIFO data write to `datax/datay/dataz/time` outside main loop after above change.

---

## 2) Item 2 — explicit double-buffered SD write path

File: `Core/Src/main.c`

### 2.1 Add append-and-swap logic in record loop
Replace per-sample direct write block:

```c
float ts_ms = (float)time[i] * 12.5f / 1000.0f;
sprintf(buffer, "%.3f %d %d %d\n", ts_ms, datax[i], datay[i], dataz[i]);
f_write(&myFile, buffer, strlen(buffer), (UINT*) &aux);
clear_buffer();
```

With:

```c
char line[LOG_LINE_MAX];
float ts_ms = (float)time[i] * 12.5f / 1000.0f;
int line_len = snprintf(line, sizeof(line), "%.3f %d %d %d\n", ts_ms, datax[i], datay[i], dataz[i]);

if (line_len > 0 && (uint32_t)line_len < LOG_LINE_MAX) {
  if ((active_log_len + (uint32_t)line_len) > LOG_CHUNK_SIZE) {
    if (flush_pending == SET) {
      dropped_lines++;
    } else {
      char *tmp = flush_log_buf;
      flush_log_buf = active_log_buf;
      flush_log_len = active_log_len;
      active_log_buf = tmp;
      active_log_len = 0U;
      flush_pending = SET;
    }
  }

  if ((active_log_len + (uint32_t)line_len) <= LOG_CHUNK_SIZE) {
    memcpy(&active_log_buf[active_log_len], line, (size_t)line_len);
    active_log_len += (uint32_t)line_len;
  } else {
    dropped_lines++;
  }
}
```

### 2.2 Add flush helper functions
In USER CODE 4, add:

```c
FRESULT flush_chunk(char *buf, uint32_t len) {
  if (len == 0U) return FR_OK;

  UINT written = 0U;
  FRESULT fr = f_write(&myFile, buf, len, &written);
  if (fr != FR_OK || written != len) {
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, TRUE);
    dbg_uartf("f_write error fr=%d written=%lu expected=%lu\n", (int)fr, (uint32_t)written, len);
    return (fr == FR_OK) ? FR_DISK_ERR : fr;
  }

  bytes_since_sync += len;
  return FR_OK;
}

FRESULT flush_pending_buffers(void) {
  FRESULT fr = FR_OK;

  if (flush_pending == SET) {
    fr = flush_chunk(flush_log_buf, flush_log_len);
    flush_pending = RESET;
    flush_log_len = 0U;
    if (fr != FR_OK) return fr;
  }

  if (active_log_len > 0U) {
    fr = flush_chunk(active_log_buf, active_log_len);
    active_log_len = 0U;
    if (fr != FR_OK) return fr;
  }

  return FR_OK;
}
```

### 2.3 Flush each loop pass when pending
In record loop, add:

```c
if (flush_pending == SET) {
  if (flush_pending_buffers() != FR_OK) {
    flag_recordData = RESET;
    flag_toggleRecord = RESET;
    flag_closeFile = SET;
  }
}
```

---

## 3) Item 3 — explicit DMA FIFO payload read implementation

File: `Core/Src/main.c`

### 3.1 Add DMA completion callbacks
In USER CODE 4 section:

```c
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    spi_fifo_dma_done = SET;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    spi_fifo_dma_error = SET;
  }
}
```

### 3.2 Add DMA read helper for FIFO burst
In USER CODE 4 section:

```c
HAL_StatusTypeDef iis_read_fifo_dma(uint8_t reg, uint8_t *dst, uint16_t len) {
  uint8_t address = (reg & 0x7F) | 0x80;

  spi_fifo_dma_done = RESET;
  spi_fifo_dma_error = RESET;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(&hspi1, &address, 1, 10) != HAL_OK) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    return HAL_ERROR;
  }

  if (HAL_SPI_Receive_DMA(&hspi1, dst, len) != HAL_OK) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    return HAL_ERROR;
  }

  uint32_t t0 = HAL_GetTick();
  while (spi_fifo_dma_done == RESET) {
    if (spi_fifo_dma_error == SET) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
      return HAL_ERROR;
    }
    if ((HAL_GetTick() - t0) > SPI_DMA_TIMEOUT_MS) {
      HAL_SPI_Abort(&hspi1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
      return HAL_TIMEOUT;
    }
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  return HAL_OK;
}
```

### 3.3 Update FIFO reader internals
In `iis_FIFO_read(...)`, replace:

```c
iis_read(0x78, WTM_THRESHOLD * 7, dataFIFO);
```

With:

```c
if (iis_read_fifo_dma(0x78, dataFIFO, WTM_THRESHOLD * 7) != HAL_OK) {
  HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, TRUE);
  dbg_uart("SPI FIFO DMA read failed\n");
  return;
}
```

---

## 4) Item 6 — explicit FatFs robustness changes (without preallocation)

File: `Core/Src/main.c`

### 4.1 Enforce write checks everywhere
All `f_write` calls must verify both status and exact byte count.
Use `flush_chunk()` for data path to centralize this check.

### 4.2 Add sync policy based on bytes + timer
In record loop after flush logic:

```c
if (bytes_since_sync >= SYNC_BYTES_THRESHOLD || time_counter_done) {
  time_counter_done = RESET;
  if (f_sync(&myFile) != FR_OK) {
    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, TRUE);
    dbg_uart("f_sync failed\n");
    flag_recordData = RESET;
    flag_toggleRecord = RESET;
    flag_closeFile = SET;
  } else {
    bytes_since_sync = 0U;
    dbg_uart("Periodic sync OK\n");
  }
}
```

### 4.3 Stop sequence must flush + sync before close
In stop branch (`~flag_toggleRecord & flag_closeFile`), before `f_close(&myFile);` add:

```c
if (flush_pending_buffers() != FR_OK) {
  dbg_uart("Final flush failed\n");
}
if (f_sync(&myFile) != FR_OK) {
  dbg_uart("Final f_sync failed\n");
}
```

### 4.4 Keep preallocation disabled
Do not add `f_expand` or any allocation-before-write call.

---

## 5) UART debug wrapping (requested explicit plan)

File: `Core/Src/main.c`

### 5.1 Add debug wrappers (compile-time gated)
In USER CODE 4 section:

```c
void dbg_uart(const char *msg) {
#if UART_DEBUG_ENABLE
  if (devMode && msg != NULL) {
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }
#else
  (void)msg;
#endif
}

void dbg_uartf(const char *fmt, ...) {
#if UART_DEBUG_ENABLE
  if (!devMode || fmt == NULL) return;

  char dbg[128];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(dbg, sizeof(dbg), fmt, args);
  va_end(args);

  if (n > 0) {
    uint16_t len = (n >= (int)sizeof(dbg)) ? (uint16_t)(sizeof(dbg) - 1U) : (uint16_t)n;
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg, len, HAL_MAX_DELAY);
  }
#else
  (void)fmt;
#endif
}
```

### 5.2 Replace direct debug UART calls
Replace all debug/trace calls:

- `send_uart2("...")` -> `dbg_uart("...")`
- `sprintf(buffer, ...); send_uart2(buffer);` -> `dbg_uartf("...", ...);`

Keep `send_uart2` only if needed for always-on messages. Otherwise remove it after migration.

### 5.3 Handcoded enable workflow

- Default in source: `#define UART_DEBUG_ENABLE 0`
- To enable debug manually: change to `#define UART_DEBUG_ENABLE 1` and rebuild.
- No runtime command/API for enabling debug.

---

## 6) Ordered implementation sequence

1. Apply section 0 and section 5 first (infrastructure and debug wrappers).
2. Apply item 1 changes (EXTI/main loop ownership).
3. Apply item 2 buffering + flush helpers.
4. Apply item 6 write/sync checks around new buffer flow.
5. Apply item 3 DMA FIFO read path.

---

## 7) Definition of done

- EXTI callback only sets event flags, does not read FIFO payload.
- SD logging is chunked and validated for full-byte writes.
- FIFO burst read uses SPI DMA + timeout/error handling.
- Sync happens on byte threshold and periodic timer event.
- UART debug is fully controlled by one handcoded compile-time flag.

