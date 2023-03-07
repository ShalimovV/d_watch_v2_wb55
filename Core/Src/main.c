/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPR121.h"
#include "lsm303c.h"
#include "ssd1306.h"
#include "ms5837_lib.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BTN_UP		5
#define BTN_DOWN	4
#define BTN_MENU	10
#define BTN_EXIT	3

#define RESET_TIME	20

#define UNDER_WATER_THRESHOLD	0.2

/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  #define VDDA_APPLI                       (3290UL)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SensorsRead */
osThreadId_t SensorsReadHandle;
const osThreadAttr_t SensorsRead_attributes = {
  .name = "SensorsRead",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for ScanBtn */
osThreadId_t ScanBtnHandle;
const osThreadAttr_t ScanBtn_attributes = {
  .name = "ScanBtn",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for Main */
osThreadId_t MainHandle;
const osThreadAttr_t Main_attributes = {
  .name = "Main",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* USER CODE BEGIN PV */
typedef struct {
float lsm303c_chip_tC;
float magneto[3];
float accel[3];
uint16_t azimuth;
char direction[3];
} mag_t;

mag_t mag = {0, {0,0,0}, {0,0,0}, 0};

RTC_TimeTypeDef sfTime;
RTC_DateTypeDef sfDate;
RTC_AlarmTypeDef sAlarm;

float vbat = 0;
uint32_t adc_data = 0;

typedef struct {
bool alarmSet_flag;
bool vibrate_on_flag;
bool sleep_flag;
bool device_on_body;
bool MS5837_is_ok;
bool MAX30102_is_ok;
bool under_water_flag;
bool lsm303_is_ok;
bool end_hrm_measure;
} watch_state_t;

watch_state_t watch_state = {0,0,0,0,0,0,0,0,0};

typedef struct {
float temperature;
float max_temp;
float min_temp;
float pressure;
float depth;
float max_depth;
uint32_t under_water_time;
uint32_t start_time;
uint32_t max_time;
uint32_t dive_counter;
float under_water_threshold;
} water_t;

water_t water = {0,0,0,0,0,0,0,0,0,0,UNDER_WATER_THRESHOLD};

uint16_t mpr121_status = 0;

typedef struct {
	bool UP;
	bool DOWN;
	bool MENU;
	bool EXIT;
}buttons_t;

buttons_t button;

typedef struct {
bool vibro_response;
} watch_setup_t;

watch_setup_t watch_setup = {1};


typedef struct {
uint8_t reset_counter;
uint8_t off_time;
bool off_flag;
uint8_t off_counter;
uint8_t enter_mnu_counter;
uint8_t sleep_counter;
int8_t page;
char buf1[20];
char buf2[20];
} display_t;

display_t display = {0, 40, false, 40, 0, 255, 0};

enum ms5837_status status;

uint8_t sensor_upd_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void SensorsRead_Task(void *argument);
void ScanBtn_Task(void *argument);
void Main_Task(void *argument);

/* USER CODE BEGIN PFP */
void scan_btn();
void read_ms5837(void);
void Vibrate(uint8_t times);
void display_off(void);
void display_on(void);
void setupTime(bool alarm);
void menu(void);
void show_bat_status(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void scan_btn() {
	MPR121_updateAll();
	if (MPR121_isNewTouch(BTN_UP))  {
		if(watch_setup.vibro_response) Vibrate(1);
		button.UP = true;
	}
	else button.UP = false;

	if (MPR121_isNewTouch(BTN_DOWN))  {
		if(watch_setup.vibro_response) Vibrate(1);
		button.DOWN = true;
	}
	else button.DOWN = false;

	if (MPR121_isNewTouch(BTN_MENU))  {
		if(watch_setup.vibro_response) Vibrate(1);
		button.MENU = true;
	}
	else button.MENU = false;

	if (MPR121_isNewTouch(BTN_EXIT))  {
		if(watch_setup.vibro_response) Vibrate(1);
		button.EXIT = true;
	}
	else button.EXIT = false;

}


void read_ms5837(void) {
	ms5837_reset();
	status = ms5837_read_temperature_and_pressure(&water.temperature, &water.pressure);
	water.depth = MS5837_depth(water.pressure);

	if (water.temperature > water.max_temp)  water.max_temp = water.temperature;
	if (water.temperature < water.min_temp)  water.min_temp = water.temperature;



	if (water.depth > water.under_water_threshold) {
		if (!watch_state.under_water_flag) water.start_time = HAL_GetTick();
		water.under_water_time = HAL_GetTick() - water.start_time;
		if (water.depth > water.max_depth) water.max_depth = water.depth;
		watch_state.under_water_flag = true;
	}
	else  {
		if ( watch_state.under_water_flag) water.dive_counter++;
		water.under_water_time = 0;
		watch_state.under_water_flag = false;
	}

	if (water.under_water_time > water.max_time) water.max_time = water.under_water_time;
}


void Vibrate(uint8_t times)
{
	for(uint8_t i=0; i < times; i++)
	{
		HAL_GPIO_WritePin(Vib_GPIO_Port, Vib_Pin, GPIO_PIN_SET);
		osDelay(100);
		//HAL_Delay(100);
		HAL_GPIO_WritePin(Vib_GPIO_Port, Vib_Pin, GPIO_PIN_RESET);
		//HAL_Delay(100);
		osDelay(200);
	}
}

void display_off(void) {
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	ssd1306_SetDisplayOn(0);
	//ssd1306_Fill(Black);
	//ssd1306_UpdateScreen();
	display.off_flag = true;
}

void display_on(void) {
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	//HAL_Delay(10);
	//ssd1306_Init();
	//ssd1306_SetContrast(255);
	ssd1306_SetDisplayOn(1);
	display.off_flag = false;
}


void setupTime(bool alarm) {
  char time[16];
  uint8_t  pos = 0, space = 0, tmpTime[3] = {sfTime.Hours, sfTime.Minutes, sfTime.Seconds};
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  while (1) {
    osDelay(200);
    if (button.EXIT) {
      osDelay(300);
      return;
    }

    if (button.UP) {
      tmpTime[pos]++;
      if (tmpTime[0] > 23) tmpTime[0] = 0;
      if (tmpTime[1] > 60) tmpTime[1] = 0;
      if (tmpTime[2] > 60) tmpTime[2] = 0;
      else space = 0;
      ssd1306_Fill(Black);
      ssd1306_UpdateScreen();
      osDelay(20);
    }

    if (button.DOWN) {
      tmpTime[pos]--;
      if (tmpTime[0] > 23) tmpTime[0] = 0;
      if (tmpTime[1] > 60) tmpTime[1] = 0;
      if (tmpTime[2] > 60) tmpTime[2] = 0;
      ssd1306_Fill(Black);
      ssd1306_UpdateScreen();
      osDelay(20);
    }

    if (button.MENU) {
      pos++;
      if (pos > 3) pos = 0;
      ssd1306_Fill(Black);
      ssd1306_UpdateScreen();
    }

    sprintf(time, "%02d:%02d:%02d", tmpTime[0], tmpTime[1], tmpTime[2]);
    ssd1306_SetCursor(0, 20);
    ssd1306_WriteString(time, Font_11x18, White);
    ssd1306_SetCursor(0, 40);
    ssd1306_WriteString("Set?", Font_11x18, White);

    if (pos == 3) {
    	ssd1306_DrawRectangle(0 , 39 , 50, 59, White);
      if (button.UP) {
       if (alarm) {
    	   sAlarm.Alarm=RTC_ALARM_A;
    	   	sAlarm.AlarmTime.Hours = tmpTime[0];
    	   	sAlarm.AlarmTime.Minutes = tmpTime[1];
    	   	sAlarm.AlarmTime.Seconds = tmpTime[2];
    	   	HAL_RTC_SetAlarm(&hrtc,&sAlarm, RTC_FORMAT_BIN);     // Включаем функцию прерывания
    	   	HAL_RTC_SetAlarm_IT(&hrtc,&sAlarm, RTC_FORMAT_BIN);  // Устанавливаем прерывание

    	    watch_state.alarmSet_flag = true;
    	     return;
        }
        else {

        		sfTime.Hours = tmpTime[0];
        	  sfTime.Minutes = tmpTime[1];
        	  sfTime.Seconds = tmpTime[2];
        	  sfTime.SubSeconds = 0x0;
        	  sfTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        	  sfTime.StoreOperation = RTC_STOREOPERATION_SET;
        	  if (HAL_RTC_SetTime(&hrtc, &sfTime, RTC_FORMAT_BIN) != HAL_OK)
        	  {
        	    Error_Handler();
        	  }
        	return;
        }
      }
    }
    else ssd1306_DrawRectangle(0 + pos * 31, 19 , 24 + pos * 31, 38, White);
    ssd1306_UpdateScreen();
  }
}




void menu(void) {
  uint8_t screen = 0, menu_cnt = 0;
  int8_t pos = 0;
  bool item[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  if (button.MENU) osDelay(300);

  while (1) {

    if (button.EXIT) return;

    if (watch_state.under_water_flag) return;

    memset(display.buf1, 0, sizeof(display.buf1));
    memset(display.buf2, 0, sizeof(display.buf2));

    ssd1306_SetCursor(20, 0);
    sprintf(display.buf1, "Menu %d %d %d", screen, pos, menu_cnt);
    ssd1306_WriteString(display.buf1, Font_7x10, White);


    if (button.UP) {
		  pos--;
		  if (pos < 0) {
			  if (screen > 0) {
				  screen--;
				  pos = 4;
			  }

		  }
		  if ((screen == 0) & (pos < 0)) pos = 0;
		  menu_cnt--;
		  ssd1306_Fill(Black);
		  ssd1306_UpdateScreen();
		  osDelay(50);
    }
    if (button.DOWN) {
		  pos++;
		  if (pos > 4) {
			  screen++;
			  pos = 0;
		  }
		  menu_cnt++;
		  ssd1306_Fill(Black);
		  ssd1306_UpdateScreen();
		  osDelay(50);
    }

    if (menu_cnt > 15) menu_cnt = 15;

    if (screen > 3) screen = 3;



    if (button.MENU) {
      item[menu_cnt] = !item[menu_cnt];
      ssd1306_Fill(Black);
      ssd1306_UpdateScreen();
      memset(display.buf1, 0, sizeof(display.buf1));
      memset(display.buf2, 0, sizeof(display.buf2));
    }

    switch (screen)
	{
    case 0:
			ssd1306_SetCursor(1, 17);
			ssd1306_WriteString("Setup time", Font_7x10, White);
			ssd1306_SetCursor(100, 17);
			if (item[0]) {
			  setupTime(false);
			  ssd1306_Fill(Black);
			  ssd1306_UpdateScreen();
			  item[0] = 0;
			}

			ssd1306_SetCursor(1, 26);
			ssd1306_WriteString("Setup alarm", Font_7x10, White);
			ssd1306_SetCursor(100, 26);
			if (item[1]) {
			  setupTime(true);
			  ssd1306_Fill(Black);
			  ssd1306_UpdateScreen();
			  item[1] = 0;
			}

			ssd1306_SetCursor(1, 35);
			ssd1306_WriteString("Dim display", Font_7x10, White);
			ssd1306_SetCursor(100, 35);
			if (item[2]) {
				ssd1306_WriteString("Yes", Font_7x10, White);
				ssd1306_SetContrast(10);
				//display_dim_flag = false;
			}
			else {
				ssd1306_WriteString("No", Font_7x10, White);
				ssd1306_SetContrast(250);
			  //display_dim_flag = true;
			}

			ssd1306_SetCursor(1, 44);
			ssd1306_WriteString("Disp off T", Font_7x10, White);
			ssd1306_SetCursor(100, 44);
			sprintf(display.buf2, "%d", display.off_time);
			ssd1306_WriteString(display.buf2, Font_7x10, White);

			if (item[3]) {
			  display.off_time += 10;
			  if (display.off_time > 60) display.off_time = 0;
			  ssd1306_Fill(Black);
			  ssd1306_UpdateScreen();
			  item[3] = !item[3];
			}

			ssd1306_SetCursor(1, 53);
			ssd1306_WriteString("Sensors upd", Font_7x10, White);
			ssd1306_SetCursor(100, 53);
			sprintf(display.buf1, "%d", sensor_upd_time);
			ssd1306_WriteString(display.buf1, Font_7x10, White);
			if (item[4]) {
				sensor_upd_time += 5;
				if (sensor_upd_time > 60 ) sensor_upd_time = 1;
				ssd1306_Fill(Black);
				ssd1306_UpdateScreen();
			  item[4] = !item[4];
			}
			break;
    case 1:
    		ssd1306_SetCursor(1, 17);
    		ssd1306_WriteString("System reset", Font_7x10, White);
    		ssd1306_SetCursor(100, 17);
    		if (item[5]) {
    				  ssd1306_WriteString("Yes?", Font_7x10, White);
    				  ssd1306_Fill(Black);
    				  ssd1306_UpdateScreen();
    				  if (button.MENU) NVIC_SystemReset();
    				  //item[5] = !item[5];
    				}
    		ssd1306_SetCursor(1, 26);
    		ssd1306_WriteString("HW info", Font_7x10, White);
    		ssd1306_SetCursor(100, 26);
    		if (item[6]) {
						ssd1306_Fill(Black);
						ssd1306_UpdateScreen();
						ssd1306_SetCursor(1, 17);
						sprintf(display.buf1, "Vbat %2.3f", vbat);
						ssd1306_WriteString(display.buf1, Font_7x10, White);
    				}

    case 2:
        		ssd1306_SetCursor(1, 17);
        		ssd1306_WriteString("Radio", Font_7x10, White);
        		ssd1306_SetCursor(100, 17);
        		ssd1306_WriteString("Off", Font_7x10, White);
        		if (item[7]) {
        				ssd1306_Fill(Black);
        			    ssd1306_UpdateScreen();
        				ssd1306_WriteString("On", Font_7x10, White);

        				item[7] = !item[7];
        				}

    	break;
    default: break;
	}

    ssd1306_DrawRectangle(0, 16 + (pos * 9), 127, 26 + (pos * 9), White);
    ssd1306_UpdateScreen();
    osDelay(200);

  }
}


void show_bat_status(void) {
	uint8_t percent = (vbat - 3.2) * 100;
	if (percent > 100) percent = 100;
	ssd1306_DrawRectangle(100,0,127,10, White);
	for (uint i = 0; i < percent / 4; i++) {
		ssd1306_Line(126 - i, 0, 126 - i, 10, White);
	}


}

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(Display_en_GPIO_Port, Display_en_Pin, GPIO_PIN_SET);
   	  ssd1306_Init();
   	  ssd1306_SetContrast(255);
       ssd1306_SetCursor(0, 0);
       ssd1306_WriteString("Display Ok", Font_7x10, White);
       ssd1306_UpdateScreen();

       ssd1306_SetCursor(0, 19);
           if(LSM303C_init()) {
           	 watch_state.lsm303_is_ok = true;
           	ssd1306_WriteString("LSM303 Ok", Font_7x10, White);
           }
           else ssd1306_WriteString("LSM303 init fail!", Font_7x10, White);

           ssd1306_SetCursor(0, 30);
           if (MPR121_begin(&hi2c1, 20, 10)){
               MPR121_setFFI(FFI_10);
               MPR121_setSFI(SFI_10);
               MPR121_setGlobalCDT(CDT_4US);  // reasonable for larger capacitances
               MPR121_autoSetElectrodeCDC_all();
               ssd1306_WriteString("MPR121 Ok", Font_7x10, White);
             	}
            else ssd1306_WriteString("MP121 init fail!", Font_7x10, White);



           ms5837_Port_Init(&hi2c1);
          	ssd1306_SetCursor(0, 52);
          	if (ms5837_is_connected() == HAL_OK) {
          		  ssd1306_WriteString("MS5837 Ok", Font_7x10, White);
          		  watch_state.MS5837_is_ok = true;
          		  read_ms5837();
          		  MS5837_set_atm_pressure(water.pressure);
          		  status = ms5837_read_temperature_and_pressure(&water.temperature, &water.pressure);
          		  water.min_temp = water.temperature;
          	  }
          	else ssd1306_WriteString("MS5837 Fail!", Font_7x10, White);

           ssd1306_UpdateScreen();

           /* Perform ADC calibration */
           	   if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
           	   {
           	     /* Calibration Error */
           	     Error_Handler();
           	   }

           HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorsRead */
  SensorsReadHandle = osThreadNew(SensorsRead_Task, NULL, &SensorsRead_attributes);

  /* creation of ScanBtn */
  ScanBtnHandle = osThreadNew(ScanBtn_Task, NULL, &ScanBtn_attributes);

  /* creation of Main */
  MainHandle = osThreadNew(Main_Task, NULL, &Main_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV4);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00301139;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Vib_Pin|Display_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Vib_Pin Display_en_Pin */
  GPIO_InitStruct.Pin = Vib_Pin|Display_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SensorsRead_Task */
/**
* @brief Function implementing the SensorsRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorsRead_Task */
void SensorsRead_Task(void *argument)
{
  /* USER CODE BEGIN SensorsRead_Task */
  /* Infinite loop */
  for(;;)
  {
	  if (!display.off_flag) {
	  			  taskENTER_CRITICAL();
	  			  /* Start ADC group regular conversion */
	  			  if (HAL_ADC_Start(&hadc1) != HAL_OK)
	  				  {
	  				  /* Error: ADC conversion start could not be performed */
	  					  Error_Handler();
	  				  }
	  			  HAL_ADC_PollForConversion(&hadc1,100);
	  			  adc_data =  HAL_ADC_GetValue(&hadc1);
	  			  HAL_ADC_Stop(&hadc1);
	  			  vbat = ((adc_data * 0.8) * 8.5) / 1000;
	  			  taskEXIT_CRITICAL();

	  			  mag.azimuth = LSM303C_getAzimuth();

	  			  mag.accel[0] = LSM303C_readAccelX();
	  			  mag.accel[1] = LSM303C_readAccelY();
	  			  mag.accel[2] = LSM303C_readAccelZ();

	  			  mag.magneto[0] = LSM303C_readMagX();
	  			  mag.magneto[1] = LSM303C_readMagY();
	  			  mag.magneto[2] = LSM303C_readMagZ();

	  			  mag.lsm303c_chip_tC = LSM303C_readTempC();

	  	}

	  	read_ms5837();

	      osDelay(1000);
  }
  /* USER CODE END SensorsRead_Task */
}

/* USER CODE BEGIN Header_ScanBtn_Task */
/**
* @brief Function implementing the ScanBtn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ScanBtn_Task */
void ScanBtn_Task(void *argument)
{
  /* USER CODE BEGIN ScanBtn_Task */
  /* Infinite loop */
  for(;;)
  {
	  HAL_IWDG_Refresh(&hiwdg);
	  scan_btn();
	  osDelay(300);
  }
  /* USER CODE END ScanBtn_Task */
}

/* USER CODE BEGIN Header_Main_Task */
/**
* @brief Function implementing the Main thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Main_Task */
void Main_Task(void *argument)
{
  /* USER CODE BEGIN Main_Task */
  /* Infinite loop */
  for(;;)
  {
	  display.off_counter--;
	  	  memset(display.buf1, 0, sizeof(display.buf1));
	  	  memset(display.buf2, 0, sizeof(display.buf2));
	  	  	  	if (display.off_counter == 0) {
	  	  	  		display_off();
	  	  	  		display.sleep_counter--;
	  	  	  	}
	  	  	  	if (!display.sleep_counter)  watch_state.sleep_flag=true;

	  	  	  	ssd1306_Fill(Black);
	  	  	  	if (!display.off_flag)show_bat_status();

	  	  	  	if ((button.UP || button.DOWN || button.MENU || button.EXIT) & display.off_flag)
	  	  	  		  	  		  			{
	  	  	  								display_on();

	  	  	  		  	  		  			display.off_counter = display.off_time;
	  	  	  		  	  		  			display.sleep_counter = 255;
	  	  	  		  	  		  			}
	  	  	  	if (button.MENU) {
	  	  	  		menu();//enter_mnu_counter++
	  	  	  	}
	  	  	  	//else enter_mnu_counter = 0;
	  	  	  	//if (enter_mnu_counter == 10) menu();

	  	  	  	if ( watch_state.alarmSet_flag) {
	  	  	  						ssd1306_SetCursor(90, 0);
	  	  	  						ssd1306_WriteString("*", Font_7x10, White);
	  	  	  					}

	  	  	  	if (button.DOWN & !display.off_flag) {
	  	  	  		display.page++;
	  	  	  		osDelay(100);
	  	  	  	}
	  	  	  	if (button.UP & !display.off_flag) {
	  	  	  		display.page--;
	  	  	  		osDelay(100);
	  	  	  	}

	  	  	  	if (display.page > 7) display.page = 7;
	  	  	  	if (display.page < 0) display.page = 0;

	  	  	  	if (button.MENU & button.EXIT) display.reset_counter++;
	  	  	  	else display.reset_counter = 0;
	  	  	  	if (display.reset_counter >= RESET_TIME) NVIC_SystemReset();

	  	  	  	HAL_RTC_GetTime(&hrtc, &sfTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	  	  	  	HAL_RTC_GetDate(&hrtc, &sfDate, RTC_FORMAT_BIN);

	  	  	  	if (!display.page &  watch_state.under_water_flag) {
	  	  	  		if (display.off_flag) display_on();
	  	  	  		display.page = 1;
	  	  	  	}


	  	  	  	switch (display.page)
	  	  	{

	  	  	  	case 0:

	  	  				ssd1306_SetCursor(0, 20);
	  	  				sprintf(display.buf2, "%02d:%02d:%02d", sfTime.Hours, sfTime.Minutes, sfTime.Seconds);
	  	  				ssd1306_WriteString(display.buf2, Font_16x26, White);
	  	  				ssd1306_SetCursor(2, 0);
	  	  				sprintf(display.buf1, "%2.1f ", water.temperature);
	  	  				ssd1306_WriteString(display.buf1, Font_7x10, White);
	  	  				sprintf(display.buf1, "%4.1f ", water.pressure / 1.333);
	  	  				ssd1306_WriteString(display.buf1, Font_7x10, White);

	  	  				ssd1306_SetCursor(0, 50);
	  	  				sprintf(display.buf1, "%02d-%02d-%d", sfDate.Date, sfDate.Month, sfDate.Year);
	  	  				ssd1306_WriteString(display.buf1, Font_7x10, White);
	  	  				switch (sfDate.WeekDay) {
	  	  				case 1: ssd1306_WriteString("  Monday", Font_7x10, White); break;
	  	  				case 2: ssd1306_WriteString("  Tuesday", Font_7x10, White); break;
	  	  				case 3: ssd1306_WriteString("  Wednesday", Font_7x10, White); break;
	  	  				case 4: ssd1306_WriteString("  Thursday", Font_7x10, White); break;
	  	  				case 5: ssd1306_WriteString("  Friday", Font_7x10, White); break;
	  	  				case 6: ssd1306_WriteString("  Saturday", Font_7x10, White); break;
	  	  				case 7: ssd1306_WriteString("  Sunday", Font_7x10, White); break;
	  	  				default: break;
	  	  				}

	  	  				break;
	  	  	  	case 1:
	  	  	  		ssd1306_SetCursor(0, 0);
	  	  	  		sprintf(display.buf2, "%02d:%02d", sfTime.Hours, sfTime.Minutes);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		switch (sfDate.WeekDay) {
	  	  	  		  				case 1: ssd1306_WriteString(" Mo", Font_7x10, White); break;
	  	  	  		  				case 2: ssd1306_WriteString(" Tu", Font_7x10, White); break;
	  	  	  		  				case 3: ssd1306_WriteString(" Wed", Font_7x10, White); break;
	  	  	  		  				case 4: ssd1306_WriteString(" Thu", Font_7x10, White); break;
	  	  	  		  				case 5: ssd1306_WriteString(" Fri", Font_7x10, White); break;
	  	  	  		  				case 6: ssd1306_WriteString(" Sat", Font_7x10, White); break;
	  	  	  		  				case 7: ssd1306_WriteString(" Sun", Font_7x10, White); break;
	  	  	  		  				default: break;
	  	  	  		  				}
	  	  	  		ssd1306_SetCursor(70, 0);
	  	  	  		sprintf(display.buf2, "%d", mag.azimuth);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);

	  	  	  		if (water.depth < 10.0) ssd1306_Line(60, 20, 60, 64, White);
	  	  	  		else ssd1306_Line(65, 20, 65, 64, White);

	  	  	  		ssd1306_SetCursor(65, 16);
	  	  	  		ssd1306_WriteString("Water T", Font_7x10, White);
	  	  	  		ssd1306_SetCursor(62, 28);
	  	  	  		if (water.depth < 10.0) sprintf(display.buf2, "%2.1f", water.temperature);
	  	  	  		else sprintf(display.buf2, "%2.0f", water.temperature);
	  	  	  		ssd1306_WriteString(display.buf2, Font_16x26, White);
	  	  	  		ssd1306_SetCursor(62, 54);
	  	  	  		sprintf(display.buf2, "%2.1f %2.1f", water.min_temp, water.max_temp);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);

	  	  	  		ssd1306_SetCursor(0, 28);
	  	  	  		if (water.depth >= 0) sprintf(display.buf1, "%1.1f", water.depth);
	  	  	  		ssd1306_WriteString(display.buf1, Font_16x26, White);
	  	  	  		ssd1306_SetCursor(0, 16);
	  	  	  		sprintf(display.buf2, "Deep %1.1f", water.max_depth);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 54);
	  	  	  		sprintf(display.buf2, "%lu", water.under_water_time /1000);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(30, 54);
	  	  	  		sprintf(display.buf2, "%lu", water.max_time /1000);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);

	  	  	  		display.off_counter++;

	  	  	  			break;
	  	  	  	case 2:
	  	  	  		ssd1306_SetCursor(0, 0);
	  	  	  		sprintf(display.buf2, "%02d:%02d %d", sfTime.Hours, sfTime.Minutes, mag.azimuth);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		switch (sfDate.WeekDay) {
	  	  	  		  	  		  				case 1: ssd1306_WriteString("  Mo", Font_7x10, White); break;
	  	  	  		  	  		  				case 2: ssd1306_WriteString("  Tu", Font_7x10, White); break;
	  	  	  		  	  		  				case 3: ssd1306_WriteString("  Wed", Font_7x10, White); break;
	  	  	  		  	  		  				case 4: ssd1306_WriteString("  Thu", Font_7x10, White); break;
	  	  	  		  	  		  				case 5: ssd1306_WriteString("  Fri", Font_7x10, White); break;
	  	  	  		  	  		  				case 6: ssd1306_WriteString("  Sat", Font_7x10, White); break;
	  	  	  		  	  		  				case 7: ssd1306_WriteString("  Sun", Font_7x10, White); break;
	  	  	  		  	  		  				default: break;
	  	  	  		  	  		  				}
	  	  	  		ssd1306_Line(64, 20, 64, 64, White);
	  	  	  		ssd1306_SetCursor(0, 20);
	  	  	  		sprintf(display.buf2, "Dep %1.1f", MS5837_depth(water.pressure));
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 32);
	  	  	  		sprintf(display.buf2, "Time %lu", water.under_water_time /1000);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 44);
	  	  	  		sprintf(display.buf2, "To %2.1f", water.temperature);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 54);
	  	  	  		sprintf(display.buf2, "Dive %lu", water.dive_counter);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);

	  	  	  		display.off_counter++;

	  	  	  			break;
	  	  	  	case 3:
	  	  	  		ssd1306_SetCursor(0, 0);
	  	  	  		sprintf(display.buf2, "%02d:%02d:%02d", sfTime.Hours, sfTime.Minutes, sfTime.Seconds);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 52);
	  	  	  		ssd1306_WriteString("Depth", Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 25);
	  	  	  		sprintf(display.buf2, "%2.3fM", MS5837_depth(water.pressure));
	  	  	  		ssd1306_WriteString(display.buf2, Font_16x26, White);
	  	  	  		display.off_counter++;

	  	  	  		break;
	  	  	  	case 4:
	  	  	  		ssd1306_SetCursor(0, 0);
	  	  	  		sprintf(display.buf2, "%02d:%02d:%02d", sfTime.Hours, sfTime.Minutes, sfTime.Seconds);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		watch_state.end_hrm_measure = true;
	  	  	  		ssd1306_SetCursor(0, 52);
	  	  	  		ssd1306_WriteString("Pressure   mbar", Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 25);
	  	  	  		sprintf(display.buf2, "%4.1f", water.pressure);
	  	  	  		ssd1306_WriteString(display.buf2, Font_16x26, White);
	  	  	  		display.off_counter++;

	  	  	  		break;

	  	  	  	case 5:

	  	  	  		ssd1306_SetCursor(0, 0);
	  	  	  		//ssd1306_WriteString("LSM303C data", Font_7x10, White);
	  	  	  		//ssd1306_SetCursor(0, 12);
	  	  	  		ssd1306_WriteString("Mag    Accel", Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0, 15);
	  	  	  		sprintf(display.buf2, "%3.3f    %3.1f", mag.magneto[0], mag.accel[0]);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0,27);
	  	  	  		sprintf(display.buf2, "%3.3f    %3.1f", mag.magneto[1], mag.accel[1]);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0,39);
	  	  	  		sprintf(display.buf2, "%3.3f    %3.1f", mag.magneto[2], mag.accel[2]);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		ssd1306_SetCursor(0,52);
	  	  	  		sprintf(display.buf2, "T chip - %3.1f", mag.lsm303c_chip_tC);
	  	  	  		ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  	  		display.off_counter++;
	  	  	  		break;

	  	  	  case 6:

	  	  		  	  ssd1306_SetCursor(0, 0);
	  	  		  	  sprintf(display.buf2, "%02d:%02d:%02d", sfTime.Hours, sfTime.Minutes, sfTime.Seconds);
	  	  		  	  ssd1306_WriteString(display.buf2, Font_7x10, White);
	  	  		  	  ssd1306_SetCursor(0, 52);
	  	  			  ssd1306_WriteString("     Compass", Font_7x10, White);
	  	  			  ssd1306_Line(32,32,42,20, White);
	  	  			  mag.azimuth = LSM303C_getAzimuth();
	  	  			  //azimuth = tmp_azimuth + 90;
	  	  			  //if (tmp_azimuth > 270) azimuth = tmp_azimuth - 270;
	  	  			  LSM303C_getDirection(mag.direction, mag.azimuth);
	  	  			  ssd1306_SetCursor(54,15);
	  	  			  sprintf(display.buf2, "Az %d", mag.azimuth);
	  	  			  ssd1306_WriteString(display.buf2, Font_7x10, White);

	  	  			  ssd1306_SetCursor(54,27);
	  	  			  //sprintf(buf2, "maxX %3.3f", max_x);
	  	  			  ssd1306_WriteString(mag.direction, Font_7x10, White);

	  	  			  display.off_counter++;
	  	  	  	  	break;


	  	  	  	default:
	  	  	  			break;

	  	  	}

	  	  	  	if (!display.off_flag) ssd1306_UpdateScreen();


	      osDelay(200);
  }
  /* USER CODE END Main_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
