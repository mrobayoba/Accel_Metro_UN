/* USER CODE BEGIN Header */
/**
 * @file           : main.c
 * @brief          : Modulo de medicion de vibraciones en ejes de tren
 * @authors        : Alejandro Narvaez, Mateo Robayo, Geronimo Nuñez
 * @project        : Accel Metro-UN
 *
 * @hardware       : STM32F411RE + IIS3DWB (SPI) + SD Card (SDIO/FatFS)
 * @sensor_odr     : 26,667 Hz (acelerometro 3 ejes, +-4g)
 * @storage_format : Binario, 10 bytes/muestra (uint32 ts + int16 x,y,z)
 * @file_rotation  : Automatica cada 20 minutos (A0, A1, A2, ...)
 *
 * @description    : El sistema lee el FIFO del IIS3DWB por polling con
 *                   WTM=20, empaqueta las muestras en formato binario en
 *                   un buffer de 50KB, y escribe a la SD en bloques
 *                   alineados a 512 bytes (sector). Los archivos rotan
 *                   automaticamente cada 20 minutos. Un preview UART
 *                   opcional (~10 Hz) permite monitorear en CoolTerm.
 *
 * Copyright (c) 2022 STMicroelectronics. All rights reserved.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- Sensor FIFO ---------- */
#define DATA_BUFFERSIZE    (20 * 7)   /* 7 bytes/word x 20 words = 140 bytes   */
#define TAG_SENSOR_ACCEL   0x02
#define TAG_SENSOR_TIMESTAMP 0x04
#define WTM_THRESHOLD      20         /* 10 pares accel+timestamp por lectura   */

/* ---------- SD write ---------- */
#define BIG_BUF_SIZE       50000      /* Buffer de acumulacion (~187 ms @ 26667 Hz) */
#define WRITE_THRESHOLD    512        /* Escribir alineado a sector SD            */
#define SYNC_EVERY_N       5000       /* f_sync cada ~5000 escrituras             */

/* ---------- Rotacion de archivos ---------- */
#define FILE_ROTATE_MS     1200000UL  /* 20 minutos en milisegundos              */

/* ---------- UART preview ---------- */
#define PREVIEW_EVERY_N    130        /* ~130 lotes x 0.75ms = ~100ms            */

/* ---------- LEDs ---------- */
#define RECORD_LED_PIN     GPIO_PIN_4
#define RECORD_LED_PORT    GPIOA
#define ERROR_LED_PIN      GPIO_PIN_0
#define ERROR_LED_PORT     GPIOC
#define BLINK_LED_PIN      GPIO_PIN_1
#define BLINK_LED_PORT     GPIOC

/* ---------- Misc ---------- */
#define BUFFER_SIZE        72         /* Buffer auxiliar para sprintf             */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * devMode: controla la salida UART para monitoreo en CoolTerm.
 *   1 = envia preview de datos (~10 Hz) + mensajes de estado
 *   0 = sin salida UART (modo produccion en el tren)
 * Para cambiar: modificar esta linea y recompilar.
 */
uint8_t devMode = 0;

/* ---------- SPI data buffers ---------- */
uint8_t data_rec[8] = { 0 };
uint8_t data_rec2[6] = { 0 };
uint8_t dataFIFO[DATA_BUFFERSIZE] = { 0 };

/* ---------- Parsed FIFO output ---------- */
int16_t datax[WTM_THRESHOLD / 2] = { 0 };
int16_t datay[WTM_THRESHOLD / 2] = { 0 };
int16_t dataz[WTM_THRESHOLD / 2] = { 0 };
uint32_t time[WTM_THRESHOLD / 2] = { 0 };

/* ---------- SD write buffer ---------- */
uint8_t bigbuf[BIG_BUF_SIZE];
uint32_t bigbuf_len = 0;

/* ---------- SD / FatFS ---------- */
FATFS myFatFS;
FIL myFile;
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/* ---------- File rotation ---------- */
char myFileName[20];
uint8_t file_index = 0;
uint32_t file_start_tick = 0;

/* ---------- Flags (volatile: modificadas por ISR) ---------- */
volatile uint8_t flag_recordData = 0;
volatile uint8_t flag_toggleRecord = 0;
volatile uint8_t flag_openFile = 0;
volatile uint8_t flag_closeFile = 0;
volatile uint8_t flag_FIFO_dataAvailable = 0;
volatile uint16_t time_counter_done = 0;

/* ---------- Counters ---------- */
uint32_t write_count = 0;
uint32_t preview_counter = 0;

/* ---------- Sensor config ---------- */
int8_t freq_fine = 0;
float ODR = 0;
int16_t x2;

/* ---------- Aux ---------- */
char buffer[BUFFER_SIZE];
uint32_t millis = 0;

int bufsize(char *buf);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void clear_buffer(void);
void iis_write(uint8_t address, uint8_t value);
void iis_read(uint8_t address, uint16_t dataSize, uint8_t *ptrDataArray);
void iis_init(void);
void iis_FIFO_read(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
        uint32_t *ptrTimestamp);
void send_uart2(char *string);
FRESULT open_new_file(void);
void close_current_file(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
    send_uart2("Starting Accel Metro-UN\n");
    HAL_Delay(100);
    iis_init();
    HAL_Delay(15);

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim11);
    send_uart2("Ready to roll!!!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /* ============================================================
         * INICIAR GRABACION
         * ============================================================ */
        if (flag_toggleRecord && flag_openFile) {
            flag_openFile = 0;

            FRESULT error = f_mount(&myFatFS, "/", 1);
            if (error == FR_OK) {
                HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, 0);
                HAL_Delay(100);
                send_uart2("SD CARD montada OK\n");

                f_getfree("", &fre_clust, &pfs);
                clear_buffer();
                total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
                sprintf(buffer, "SD Total: %lu KB\n", total);
                send_uart2(buffer);
                clear_buffer();
                free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
                sprintf(buffer, "SD Libre: %lu KB\n\n", free_space);
                send_uart2(buffer);
                clear_buffer();

                file_index = 0;
                error = open_new_file();

                if (error == FR_OK) {
                    HAL_GPIO_WritePin(RECORD_LED_PORT, RECORD_LED_PIN, 1);
                    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, 0);

                    iis_write(0x10, 0b10101000); /* Accel ON, +-4g */
                    HAL_Delay(10);

                    flag_recordData = 1;
                    flag_closeFile = 1;

                    iis_write(0x42, 0xAA);       /* Reset timestamp */
                    iis_write(0x0A, 0b01000000); /* Clear FIFO */
                    iis_write(0x0A, 0b01000110); /* FIFO continuous mode */

                    write_count = 0;
                    preview_counter = 0;
                    bigbuf_len = 0;
                    file_start_tick = HAL_GetTick();

                    iis_read(0x63, 1, (uint8_t*)&freq_fine);
                    ODR = 26667 * (1 + (0.0015 * freq_fine));

                    millis = HAL_GetTick();
                    clear_buffer();
                    sprintf(buffer, "Grabando desde t=%lu ms\n", millis);
                    send_uart2(buffer);
                    clear_buffer();

                } else {
                    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, 1);
                    send_uart2("ERROR: no se pudo abrir archivo\n");
                }

                HAL_TIM_Base_Stop(&htim11);
                HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, 0);
                HAL_TIM_Base_Start(&htim5);
                HAL_Delay(100);

            } else {
                HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, 1);
                clear_buffer();
                sprintf(buffer, "ERROR SD mount: %i\n", error);
                send_uart2(buffer);
            }

        /* ============================================================
         * DETENER GRABACION
         * ============================================================ */
        } else if (!flag_toggleRecord && flag_closeFile) {
            flag_closeFile = 0;
            flag_recordData = 0;
            HAL_GPIO_WritePin(RECORD_LED_PORT, RECORD_LED_PIN, 0);
            HAL_TIM_Base_Stop(&htim5);
            HAL_TIM_Base_Start(&htim11);

            if (bigbuf_len > 0) {
                uint32_t aux = 0;
                f_write(&myFile, bigbuf, bigbuf_len, (UINT*)&aux);
                bigbuf_len = 0;
            }

            close_current_file();

            millis = HAL_GetTick();
            clear_buffer();
            sprintf(buffer, "Cerrado t=%lu ms\n", millis);
            send_uart2(buffer);
            clear_buffer();
        }

        /* ============================================================
         * LOOP DE GRABACION
         * ============================================================ */
        if (flag_recordData) {

            /* --- PASO 1: Leer FIFO --- */
            iis_FIFO_read(datax, datay, dataz, time);

            /* --- PASO 2: Empaquetar binario en bigbuf --- */
            for (uint16_t i = 0; i < WTM_THRESHOLD / 2; i++) {
                if (time[i] == 0) continue;
                if (bigbuf_len + 10 <= BIG_BUF_SIZE) {
                    bigbuf[bigbuf_len++] = (uint8_t)(time[i]);
                    bigbuf[bigbuf_len++] = (uint8_t)(time[i] >> 8);
                    bigbuf[bigbuf_len++] = (uint8_t)(time[i] >> 16);
                    bigbuf[bigbuf_len++] = (uint8_t)(time[i] >> 24);
                    bigbuf[bigbuf_len++] = (uint8_t)(datax[i]);
                    bigbuf[bigbuf_len++] = (uint8_t)(datax[i] >> 8);
                    bigbuf[bigbuf_len++] = (uint8_t)(datay[i]);
                    bigbuf[bigbuf_len++] = (uint8_t)(datay[i] >> 8);
                    bigbuf[bigbuf_len++] = (uint8_t)(dataz[i]);
                    bigbuf[bigbuf_len++] = (uint8_t)(dataz[i] >> 8);
                }
            }

            /* --- PASO 3: Preview UART para CoolTerm --- */
            preview_counter++;
            if (devMode && preview_counter >= PREVIEW_EVERY_N) {
                preview_counter = 0;
                for (uint16_t i = 0; i < WTM_THRESHOLD / 2; i++) {
                    if (time[i] != 0) {
                        float ts_ms = (float)time[i] * 12.5f / 1000.0f;
                        float ax = (float)datax[i] * 0.122f;
                        float ay = (float)datay[i] * 0.122f;
                        float az = (float)dataz[i] * 0.122f;
                        sprintf(buffer, "%.2f,%.2f,%.2f,%.2f\n", ts_ms, ax, ay, az);
                        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 10);
                        clear_buffer();
                        break;
                    }
                }
            }

            /* --- PASO 4: Escribir a SD alineado a 512 bytes --- */
            if (bigbuf_len >= WRITE_THRESHOLD) {
                uint32_t aux = 0;
                uint32_t to_write = (bigbuf_len / 512) * 512;
                FRESULT wr_err = f_write(&myFile, bigbuf, to_write, (UINT*)&aux);

                if (wr_err != FR_OK || aux != to_write) {
                    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, 1);
                    send_uart2("ERR:SD_WR\n");
                }

                uint32_t remainder = bigbuf_len - to_write;
                if (remainder > 0) {
                    memmove(bigbuf, bigbuf + to_write, remainder);
                }
                bigbuf_len = remainder;
                write_count++;
            }

            /* --- PASO 5: f_sync periodico --- */
            if (write_count >= SYNC_EVERY_N) {
                write_count = 0;
                f_sync(&myFile);
            }

            /* --- PASO 6: f_sync por timer de 30 min --- */
            if (time_counter_done) {
                time_counter_done = 0;
                f_sync(&myFile);
            }

            /* --- PASO 7: Rotacion de archivo cada 20 min --- */
            if ((HAL_GetTick() - file_start_tick) >= FILE_ROTATE_MS) {
                if (bigbuf_len > 0) {
                    uint32_t aux = 0;
                    f_write(&myFile, bigbuf, bigbuf_len, (UINT*)&aux);
                    bigbuf_len = 0;
                }
                f_sync(&myFile);
                f_close(&myFile);

                file_index++;
                FRESULT err = open_new_file();
                if (err != FR_OK) {
                    HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, 1);
                    send_uart2("ERR: rotacion fallida\n");
                    flag_recordData = 0;
                } else {
                    file_start_tick = HAL_GetTick();
                    write_count = 0;
                    clear_buffer();
                    sprintf(buffer, "Rotado: %s\n", myFileName);
                    send_uart2(buffer);
                    clear_buffer();
                }
            }
        }
    }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ================================================================
 * open_new_file() - Abre rec_A{N}.bin con header binario
 * ================================================================ */
FRESULT open_new_file(void) {
    sprintf(myFileName, "rec_A%u.bin", file_index);
    FRESULT err = f_open(&myFile, myFileName, FA_WRITE | FA_CREATE_ALWAYS);
    if (err != FR_OK) return err;

    f_puts("BINARY FORMAT: 10 bytes per sample\n", &myFile);
    f_puts("Each sample: uint32_t timestamp(LSB), int16_t X, int16_t Y, int16_t Z\n", &myFile);
    f_puts("Timestamp: 1 LSB = 12.5us, Accel: 1 LSB = 0.122mg, +-4g\n", &myFile);
    clear_buffer();
    sprintf(buffer, "WTM=%d ODR~26667Hz file_index=%u\n", (uint16_t)WTM_THRESHOLD, file_index);
    f_puts(buffer, &myFile);
    clear_buffer();
    f_puts("DATA_START\n", &myFile);

    return FR_OK;
}

/* ================================================================
 * close_current_file() - Cierra archivo, apaga accel, desmonta SD
 * ================================================================ */
void close_current_file(void) {
    f_close(&myFile);
    iis_write(0x10, 0b00001000);
    iis_write(0x0A, 0b01000000);

    FRESULT error = f_mount(NULL, "/", 1);
    HAL_Delay(100);
    if (error == FR_OK) send_uart2("SD desmontada OK\n");
    HAL_Delay(10);
    HAL_TIM_Base_Start(&htim5);
}

/* ================================================================
 * Interrupciones
 * ================================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        if (flag_toggleRecord == 0) {
            flag_toggleRecord = 1;
            flag_openFile = 1;
        } else {
            flag_toggleRecord = 0;
        }
    }
    if (GPIO_Pin == GPIO_PIN_9) {
        iis_FIFO_read(datax, datay, dataz, time); /* Lectura en ISR (original) */
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM5) {
        if (flag_recordData) time_counter_done = 1;
    }
    if (htim->Instance == TIM11) {
        HAL_GPIO_TogglePin(BLINK_LED_PORT, BLINK_LED_PIN);
    }
}

/* ================================================================
 * Funciones auxiliares
 * ================================================================ */
int bufsize(char *buf) {
    int i = 0;
    while (*buf++ != '\0') i++;
    return i;
}

void clear_buffer(void) {
    memset(buffer, 0, BUFFER_SIZE);
}

/* ================================================================
 * IIS3DWB - SPI
 * ================================================================ */
void iis_write(uint8_t address, uint8_t value) {
    uint8_t data[2];
    data[0] = address & 0x7F;
    data[1] = value;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void iis_read(uint8_t address, uint16_t dataSize, uint8_t *ptrDataArray) {
    address = (address & 0x7F) | 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &address, 1, 10);
    HAL_SPI_Receive(&hspi1, ptrDataArray, dataSize, 10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
 * @brief  Inicializa el IIS3DWB: reset, timestamp, FIFO continuo con WTM.
 */
void iis_init(void) {
    uint16_t auxVal = WTM_THRESHOLD & 0xFF;
    uint16_t auxMask = WTM_THRESHOLD >> 8;

    iis_write(0x12, 0b00000011); /* Software reset */
    HAL_Delay(10);

    iis_read(0x0F, 1, data_rec);
    x2 = data_rec[0];
    clear_buffer();
    sprintf(buffer, "IIS3DWB ID: %i\n", x2);
    send_uart2(buffer);
    clear_buffer();
    send_uart2("Configurando acelerometro...\n");
    iis_read(0x10, 1, data_rec);

    iis_write(0x10, 0b00000000); /* CTRL1_XL: accel OFF          */
    iis_write(0x15, 0b00000000); /* CTRL6: 3 ejes default        */
    iis_write(0x17, 0b00000000); /* CTRL8: BW 6.3 kHz            */
    iis_write(0x19, 0b00100000); /* CTRL10: timestamp ON          */
    iis_write(0x0A, 0b00000000); /* FIFO_CTRL4: reset FIFO        */
    iis_write(0x07, (uint8_t)auxVal);
    iis_write(0x08, (uint8_t)auxMask); /* WTM threshold           */
    iis_write(0x09, 0b00001010); /* FIFO_CTRL3: BDR 26667 Hz      */
    iis_write(0x0A, 0b01000110); /* FIFO_CTRL4: continuous mode   */
}

/**
 * @brief  Lee FIFO del IIS3DWB y extrae pares accel+timestamp.
 *
 * Polling del registro 0x3B esperando WTM_IA (bit 7).
 * Lee WTM_THRESHOLD * 7 bytes y parsea tags 0x02 (accel) y 0x04 (timestamp).
 * Timeout de 1000 intentos para evitar bloqueo infinito.
 */
void iis_FIFO_read(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
        uint32_t *ptrTimestamp) {

    uint32_t retries = 0;
    while (!(flag_FIFO_dataAvailable >> 7)) {
        iis_read(0x3B, 1, &flag_FIFO_dataAvailable);
        if (++retries > 1000) return;
    }
    flag_FIFO_dataAvailable = 0;

    iis_read(0x78, WTM_THRESHOLD * 7, dataFIFO);

    int16_t  temp_x = 0, temp_y = 0, temp_z = 0;
    uint32_t temp_ts = 0;
    uint8_t  has_accel = 0, has_ts = 0;
    uint16_t out_idx = 0;

    for (uint16_t j = 0; j < (uint16_t)(WTM_THRESHOLD / 2); j++) {
        ptrDataX[j] = 0;
        ptrDataY[j] = 0;
        ptrDataZ[j] = 0;
        ptrTimestamp[j] = 0;
    }

    for (uint16_t i = 0; i < WTM_THRESHOLD; i++) {
        uint8_t tag = dataFIFO[i * 7] >> 3;

        if (tag == TAG_SENSOR_ACCEL) {
            temp_x = (int16_t)((dataFIFO[i*7+2] << 8) | dataFIFO[i*7+1]);
            temp_y = (int16_t)((dataFIFO[i*7+4] << 8) | dataFIFO[i*7+3]);
            temp_z = (int16_t)((dataFIFO[i*7+6] << 8) | dataFIFO[i*7+5]);
            has_accel = 1;
        }
        else if (tag == TAG_SENSOR_TIMESTAMP) {
            temp_ts = ((uint32_t)dataFIFO[i*7+4] << 24)
                    | ((uint32_t)dataFIFO[i*7+3] << 16)
                    | ((uint32_t)dataFIFO[i*7+2] << 8)
                    |  (uint32_t)dataFIFO[i*7+1];
            has_ts = 1;
        }

        if (has_accel && has_ts && out_idx < (uint16_t)(WTM_THRESHOLD / 2)) {
            ptrDataX[out_idx] = temp_x;
            ptrDataY[out_idx] = temp_y;
            ptrDataZ[out_idx] = temp_z;
            ptrTimestamp[out_idx] = temp_ts;
            out_idx++;
            has_accel = 0;
            has_ts = 0;
        }
    }
}

/**
 * @brief  Transmite string por UART2 si devMode activo.
 */
void send_uart2(char *string) {
    if (!devMode) return;
    uint16_t len = strlen(string);
    HAL_UART_Transmit(&huart2, (uint8_t*)string, len, HAL_MAX_DELAY);
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
