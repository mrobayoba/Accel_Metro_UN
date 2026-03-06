/* USER CODE BEGIN Header */
/**
 * Este es el código que tiene comunicación SPI para el adxl
 * @Authors 	   : Alejandro Narvaez, Mateo Robayo
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
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
#include "string.h"
#include "stdio.h"
#include "math.h"
//#include "micros.h"
#include <stdbool.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//HAL_TickFreqTypeDef freq = 0U; MAYBE BUT NOT NOW
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_DEBUG_ENABLE 1   // Handcoded: set to 1 to enable UART logs, 0 to compile out debug logs

#define BUFFER_SIZE 72
#define DATA_BUFFERSIZE 20*7 // 7 bytes per word * 20 words
#define BUFFER_DATA_SIZE 10000

#define LOG_CHUNK_SIZE       4096U
#define LOG_LINE_MAX         64U
#define SYNC_BYTES_THRESHOLD (256U * 1024U)
#define SPI_DMA_TIMEOUT_MS   20U

#define TAG_SENSOR_ACCEL 0x02
#define TAG_SENSOR_TIMESTAMP 0x04
#define WTM_THRESHOLD 20 // 10 accel+timestamp pairs per batch

#define TRUE 1
#define FALSE 0

#define RECORD_LED_PIN GPIO_PIN_4
#define RECORD_LED_PORT GPIOA

#define ERROR_LED_PIN GPIO_PIN_0
#define ERROR_LED_PORT GPIOC

#define BLINK_LED_PIN GPIO_PIN_1
#define BLINK_LED_PORT GPIOC


/* Frecuencia objetivo de muestreo - CAMBIAR ESTE VALOR SEGÚN NECESIDAD
 * Ejemplos:
 *   1.0f  → 1 muestra cada 1ms    (1000 Hz)
 *   0.5f  → 1 muestra cada 0.5ms  (2000 Hz)
 *   2.0f  → 1 muestra cada 2ms    (500  Hz)
 *   0.1f  → 1 muestra cada 0.1ms  (10000 Hz)
 * Mínimo posible: 0.0375ms (26667 Hz, ODR nativo del sensor)
 */
#define TARGET_PERIOD_MS  0.5f
// Conversión a LSB de timestamp (1 LSB = 12.5us = 0.0125ms)
#define TS_THRESHOLD  ((uint32_t)(TARGET_PERIOD_MS / 0.0125f))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//variables para almacenar los datos del acelerometro y el encoder
uint8_t devMode = TRUE;
uint8_t data_rec[8] = { 0 };
uint8_t data_rec2[6] = { 0 };
uint8_t rec[5] = { 0 };

/* For store the data 16bit from accelerometer just before write it into the txt file */
uint8_t dataFIFO[DATA_BUFFERSIZE] = { 0 };
char bufferData[BUFFER_DATA_SIZE];

int16_t datax[WTM_THRESHOLD / 2] = { 0 };
int16_t datay[WTM_THRESHOLD / 2] = { 0 };
int16_t dataz[WTM_THRESHOLD / 2] = { 0 };
uint32_t time[WTM_THRESHOLD / 2] = { 0 };

int8_t freq_fine = 0;
float ODR = 0;

uint32_t millis = 0;

uint8_t check[1];
int16_t x, y, z, x2, z2;
int16_t y2;
uint32_t encoder;
uint32_t RPM;

float xg, yg, zg;
int16_t gyrox, gyroy, gyroz, temp;

double ang_x_prev, ang_y, ang_y_prev, ang_x, angx, angy, angz;
double VgirosX, VgirosY, VgirosXcum, VgirosYcum;

uint32_t dt, timePrev;
volatile uint16_t time_counter_done = RESET; // threshold time flag
// banderas de las interrupciones
volatile uint8_t flag_recordData = RESET; // acquire and record flag
volatile uint8_t flag_toggleRecord = RESET; // start/stop recording flag
volatile uint8_t flag_openFile = RESET; // open file flag
volatile uint8_t flag_closeFile = RESET; // close file flag
uint8_t flag_dataAvailable = RESET; // SETs by read the status register XLDA bit
uint8_t flag_FIFO_dataAvailable = RESET; // SETs by read FIFO_WTM_IA flag on FIFO_status register 2

volatile uint8_t flag_fifo_irq = RESET;

volatile uint8_t spi_fifo_dma_done = RESET;
volatile uint8_t spi_fifo_dma_error = RESET;

// estructuras y variables para el manejo de la sd
FATFS myFatFS;
FIL myFile;
uint32_t myBytes;
uint32_t br, bw;
uint8_t firstTimeOpen = TRUE;

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

char buffer[BUFFER_SIZE];  // to store strings..
char myFileName[15] = "record_1.txt";

int data_b[1];
int i = 0;

// Double-buffered log writing
char log_buf_a[LOG_CHUNK_SIZE] = {0};
char log_buf_b[LOG_CHUNK_SIZE] = {0};
char *active_log_buf = log_buf_a;
char *flush_log_buf  = log_buf_b;
uint32_t active_log_len = 0U;
uint32_t flush_log_len = 0U;
volatile uint8_t flush_pending = RESET;

uint32_t bytes_since_sync = 0U;
uint32_t dropped_lines = 0U;

/* Decimation counter: applied per raw accel sample inside FIFO read */
uint8_t decim_counter = 0;
uint32_t accumulated_samples = 0;
uint32_t last_saved_ts = 0; // timestamp de la ultima muestra guardada (en LSB de 12.5us)

int bufsize(char *buf);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void clear_buffer(void);

// Debug wrapper functions
void dbg_uart(const char *msg);
void dbg_uartf(const char *fmt, ...);

// Buffering and flushing
HAL_StatusTypeDef iis_read_fifo_dma(uint8_t reg, uint8_t *dst, uint16_t len);
FRESULT flush_chunk(char *buf, uint32_t len);
FRESULT flush_pending_buffers(void);

//funciones del acelerometro
void iis_write(uint8_t address, uint8_t value);
void iis_read(uint8_t address, uint16_t dataSize, uint8_t *ptrDataArray);
void iis_init(void);

/* NOT DEFINED YET*/
void iis_normal_read(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
		uint32_t *ptrTimestamp);
void iis_FIFO_read(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
		uint32_t *ptrTimestamp);

void dataBuffering(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
		uint32_t *ptrTimestamp, char *ptrBuffer, char *ptrStrData);
void clear_string(char *string);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	dbg_uart("Starting Accel Metro-UN\n");
	HAL_Delay(100);
	iis_init();
	HAL_Delay(15);

	HAL_TIM_Base_Start_IT(&htim5); // Flush timer
	HAL_TIM_Base_Start_IT(&htim11); // Blinky timer
	dbg_uart("Ready to roll!!!");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (flag_toggleRecord & flag_openFile) { // flag_toggleRecord: grabacion
			flag_openFile = RESET;
			FRESULT error = f_mount(&myFatFS, "/", 1);
			if (error == FR_OK) { // To start data recording...
				HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, FALSE);
				HAL_Delay(100);
				dbg_uart("SD CARD montada satisfactoriamiente\n");
				/*************** Card capacity details ********************/
				/* Check free space */
				f_getfree("", &fre_clust, &pfs);
				clear_buffer();
				total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
				sprintf(buffer, "SD CARD Total Size: \t%lu\n", total);
				dbg_uart(buffer);

				clear_buffer();
				free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
				sprintf(buffer, "SD CARD Free Space: \t%lu\n\n", free_space);
				dbg_uart(buffer);
				clear_buffer();

				millis = HAL_GetTick();
				sprintf(buffer, "%iA", (int) millis);

				dbg_uart(buffer);
				dbg_uart("\n");
				clear_buffer();

				HAL_Delay(100);
				BYTE openMode = FA_WRITE | FA_OPEN_APPEND;
				if (firstTimeOpen) {
					openMode = FA_WRITE | FA_CREATE_ALWAYS;
				}

				error = f_open(&myFile, myFileName, openMode);
				if (error == FR_OK) {
					HAL_GPIO_WritePin(RECORD_LED_PORT, RECORD_LED_PIN, TRUE);
					HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, FALSE);
					dbg_uart("Archivo abierto\n\n");

					if (firstTimeOpen) {
						firstTimeOpen = FALSE;
						f_puts("IIS3DWB 3-axis accelerometer by STM32\n",
								&myFile);
						sprintf(buffer,
								"Configuration: FIFO continuous mode WTM = %d Sensibility = +-4g\n",
								(uint16_t) WTM_THRESHOLD);
						f_puts(buffer, &myFile);
						clear_buffer();
						f_puts("Timestamp resolution = 12.5us/LSB converted to ms, accel resolution 0.122mg/LSB\n", &myFile);
						sprintf(buffer, "Target sampling period = %.4f ms (%.1f Hz)\n\n",
								(float)TARGET_PERIOD_MS, 1000.0f / TARGET_PERIOD_MS);
						f_puts(buffer, &myFile);
						clear_buffer();
						f_puts("Timestamp(ms) accelX(mg) accelY(mg) accelZ(mg)\n", &myFile);
					}

					sprintf(buffer,
							"Comenzamos a adquirir datos, tiempo (host): ");
					dbg_uart(buffer);
					clear_buffer();
					millis = HAL_GetTick();
					sprintf(buffer, "%iA\n", (int) millis);
					dbg_uart(buffer);
					clear_buffer();

					// Enable accelerometer just before start measuring
					iis_write(0x10, 0b10101000); // accelerometer enabled, +-4g

					HAL_Delay(10); // necessary before start data reading...

					flag_recordData = SET; // Start data acquisition and recording...
					flag_closeFile = SET; // order to close file

					iis_write(0x42, 0xAA); // To reset timestamp counter to zero
					iis_write(0x0A, 0b01000000); // to clear FIFO just before start data recording
					iis_write(0x0A, 0b01000110); // to start FIFO as continuous mode

					decim_counter = 0;
					accumulated_samples = 0;
					last_saved_ts = 0;

//This is to get the accel ODR
					iis_read(0x63, 1, (uint8_t*) &freq_fine);
					ODR = 26667 * (1 + (0.0015 * freq_fine)); // to get the "real" ODR

					// Do NOT write the ODR line (was causing the corrupt first timestamp)
					// sprintf(buffer, "%.2f 0 0 0 \n", ODR);
					// f_puts(buffer, &myFile);
					clear_buffer();

				} else {
					HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, TRUE);
					dbg_uart(
							"El archivo no pudo ser abierto, presiona el boton reset...\n\n");
				}

				HAL_TIM_Base_Stop(&htim11);
				HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, FALSE);
				HAL_TIM_Base_Start(&htim5);
				HAL_Delay(100);

			} else {
				HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, TRUE);
				clear_buffer();
				dbg_uart("ERROR!!! in mounting SD CARD...\n");
				sprintf(buffer, "Numero de error: %i \n\n", error);
				dbg_uart(buffer);

			}
		} else if (~flag_toggleRecord & flag_closeFile) { // To stop data recording...
			flag_closeFile = RESET; //Acknowledge
			flag_recordData = RESET;
			HAL_GPIO_WritePin(RECORD_LED_PORT, RECORD_LED_PIN, FALSE);
			/* Close file */
			HAL_TIM_Base_Stop(&htim5);
			HAL_TIM_Base_Start(&htim11);

			clear_buffer();
			millis = HAL_GetTick();
			millis = HAL_GetTick();
			sprintf(buffer, "Archivo cerrado en el tiempo: %iB", (int) millis);

			dbg_uart(buffer);
			dbg_uart("\n");
			clear_buffer();

			// Flush remaining data before closing
			if (flush_pending_buffers() != FR_OK) {
				dbg_uart("Final flush failed\n");
			}
			if (f_sync(&myFile) != FR_OK) {
				dbg_uart("Final f_sync failed\n");
			}

			f_close(&myFile);

			// Disable accelerometer after finish data recording
			iis_write(0x10, 0b00001000); // accelerometer disabled, +-4g
			iis_write(0x0A, 0b01000000); // to clear FIFO

			/* Unmount SDCARD */
			FRESULT error = f_mount(NULL, "/", 1);
			HAL_Delay(100);
			if (error == FR_OK)
				dbg_uart("SD CARD UNMOUNTED successfully...\n");

			HAL_Delay(10);

			HAL_TIM_Base_Start(&htim5);

		}

		if (flag_recordData) {
			if (flag_fifo_irq) {
				flag_fifo_irq = RESET;
			dbg_uart("Reading FIFO\n");
			iis_FIFO_read(datax, datay, dataz, time);
		}

		for (uint16_t i = 0; i < WTM_THRESHOLD / 2; i++) {				
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
			}

			if (flush_pending == SET) {
				if (flush_pending_buffers() != FR_OK) {
					flag_recordData = RESET;
					flag_toggleRecord = RESET;
					flag_closeFile = SET;
				}
			}

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
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//interrupciones EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) { // Button_pin
		switch (flag_toggleRecord) { // To start/stop recording
		case 0:
			flag_toggleRecord = SET;
			flag_openFile = SET; // To open file, for recording
			break;
		case 1:
			flag_toggleRecord = 0;
			break;
		default:
			__NOP();
		}
	}
	if (GPIO_Pin == GPIO_PIN_9) { // Accel FIFO watermark IRQ
		flag_fifo_irq = SET;
		dbg_uart("IRQ!\n");
	}
}

//interrupciones TIM
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM5) { // 30min timer
		if (flag_recordData) time_counter_done = SET; // threshold time reached
	}

	if (htim->Instance == TIM11) { // 100ms timer
		HAL_GPIO_TogglePin(BLINK_LED_PORT, BLINK_LED_PIN);
	}

}

// User defined functions

int bufsize(char *buf) {
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

void clear_buffer(void) {
	for (int i = 0; i < BUFFER_SIZE; i++) {
		buffer[i] = '\0';
	}
}

// Debug wrapper functions (compile-time gated)
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

// Buffered flush functions
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

//funciones del acelerometro
void iis_write(uint8_t address, uint8_t value) {
	uint8_t data[2];
	data[0] = address & 0x7F;  // write
	data[1] = value;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // pull the cs pin low
	HAL_SPI_Transmit(&hspi1, data, 2, 100);  // write data to register
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // pull the cs pin high
}

void iis_read(uint8_t address, uint16_t dataSize, uint8_t *ptrDataArray) {
	address = (address & 0x7F) | 0x80;   // read operation

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // pull the pin low
	HAL_SPI_Transmit(&hspi1, &address, 1, 10);  // send address
	HAL_SPI_Receive(&hspi1, ptrDataArray, dataSize, 10); // receive 6 bytes data
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // pull the pin high
}

void iis_init(void) // look for the registers who are being modified
{
	uint16_t auxVal = 0;
	uint16_t auxMask = 0;

	auxMask = 0b0000000011111111;
	auxVal = WTM_THRESHOLD & auxMask;
	auxMask = WTM_THRESHOLD >> 8;

	iis_write(0x12, 0b00000011); // Software reset (Reset device) use one time and then comment back this line!!!
	HAL_Delay(10);

	iis_read(0x0F, 1, data_rec);
	x2 = data_rec[0];

	clear_buffer();

	sprintf(buffer, "IIS accel ID: ");
	dbg_uart(buffer);
	clear_buffer();
	sprintf(buffer, "%i\n", x2);
	dbg_uart(buffer);
	dbg_uart("Configuring accel...\n");
	iis_read(0x10, 1, data_rec);

	iis_write(0x10, 0b00000000); // clear accel control register 1
	iis_write(0x15, 0b00000000); // clear control register 6 and set 3 axis default mode
	iis_write(0x17, 0b00000000); // clear control register 8 and set bandwidth at 6.3kHz
	iis_write(0x19, 0b00100000); // control register 10 timestamp counter enable
	iis_write(0x0A, 0b00000000); // FIFO control register 4 - FIFO Clear and Reset
	iis_write(0x07, (uint8_t) auxVal);
	iis_write(0x08, (uint8_t) auxMask); // FIFO control register 1 and 2 - Set Watermark Threshold
	iis_write(0x09, 0b00001010); // FIFO control register 3 - Sets and enables write freq in FIFO at 26667Hz
	iis_write(0x0A, 0b01000110); // FIFO control register 4 - continuous mode

	// Configure interrupt pin characteristics (CTRL3_C = 0x12)
	// H_LACTIVE=0 (active high), PP_OD=0 (push-pull)
	iis_write(0x12, 0b01000100);

	// Route FIFO watermark interrupt to INT1 pin (INT1_CTRL = 0x0D)
	iis_write(0x0D, 0b00001000); // bit 3: FIFO_TH enable

	dbg_uart("Interrupt routing configured\n");
}

void iis_normal_read(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
		uint32_t *ptrTimestamp) {
	while (!(flag_dataAvailable & 0b00000001)) {
		iis_read(0x1E, 1, &flag_dataAvailable);
	}
	iis_read(0x78, 6, data_rec);
	iis_read(0x40, 4, data_rec2);

	datax[0] = (data_rec[1] << 8) | data_rec[0];
	datay[0] = (data_rec[3] << 8) | data_rec[2];
	dataz[0] = (data_rec[5] << 8) | data_rec[4];

	time[0] = (data_rec2[3] << 8 * 3) | (data_rec2[2] << 8 * 2)
			| (data_rec2[1] << 8) | (data_rec2[0] << 0);
}

/**
 * @brief  Read FIFO and store all complete accel+timestamp pairs in order.
 */
void iis_FIFO_read(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
		uint32_t *ptrTimestamp) {

	while (!(flag_FIFO_dataAvailable >> 7)) {
		iis_read(0x3B, 1, &flag_FIFO_dataAvailable);
	}
	flag_FIFO_dataAvailable = RESET;

	if (iis_read_fifo_dma(0x78, dataFIFO, WTM_THRESHOLD * 7) != HAL_OK) {
		HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, TRUE);
		dbg_uart("SPI FIFO DMA read failed\n");
		return;
	}

	int16_t  temp_x = 0, temp_y = 0, temp_z = 0;
	uint32_t temp_ts = 0;
	uint8_t  has_accel = 0, has_ts = 0;
	uint16_t out_idx = 0;

	// Zero output arrays
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

void dataBuffering(int16_t *ptrDataX, int16_t *ptrDataY, int16_t *ptrDataZ,
		uint32_t *ptrTimestamp, char *ptrBuffer, char *ptrStrData) {
	for (uint16_t i = 0; i < WTM_THRESHOLD / 2; i++) {
		float ts_ms = (float)(*ptrTimestamp) * 12.5f / 1000.0f;
		sprintf(ptrBuffer, "%.3f %d %d %d \n", ts_ms, *ptrDataX, *ptrDataY, *ptrDataZ);
		strcat(ptrStrData, ptrBuffer);
		clear_buffer();
		ptrDataX++;
		ptrDataY++;
		ptrDataZ++;
		ptrTimestamp++;
	}
}

void clear_string(char *string) {
	for (int i = 0; i < sizeof(string); i++) {
		string[i] = '\0';
	}
}

// SPI DMA callbacks for FIFO burst read
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

// DMA-assisted FIFO burst read
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
