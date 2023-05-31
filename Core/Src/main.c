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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmpsensor.h"
#include "scheduler.h"
#include <stdio.h>
#include <string.h>
#include "../ECUAL/SERVO/SERVO.h"
#include <stdint.h>
#include<math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SERVO_Motor1  0
#define SERVO_Motor2  1
#define SERVO_Motor3  2
#define SERVO_Motor4  3
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t var=0, Light,TEMP;

#define RX_BUFFER_SIZE 50 // độ dài tối đa của bộ đệm đ�?c UART
uint8_t rx_buffer[RX_BUFFER_SIZE]; // khai báo bộ đệm đ�?c UART
uint8_t rx_data; // biến lưu trữ ký tự nhận được từ UART
uint16_t rx_index = 0; // biến đếm số ký tự đã nhận được
uint16_t value[4];
uint8_t rx,status,status_ack,status_servo;
char stringTemp[20];
char string[20];
uint16_t ma[3];
uint8_t couter=0,precouter=0;
unsigned int  verz = 0;
long previousMillis1 = 0;
long previousMillis2 = 0;
long previousMillis3 = 0;
long previousMillis4 = 0;
long previousMicros = 0;
unsigned long currentMillis=0;
unsigned long currentMicros =0;
int count0, arrayStep, arrayMax, countverz, Taster, stepsMax, steps, time = 1, del = 1, tempp;
int dif[4], ist[4], sol[4],  dir[4]; // difference between stored position and momentary position
int joint0[180];// array for servo(s)
int joint1[180];
int joint2[180];
int joint3[180];
int top = 179; // we should not write over the end from a array
int Delay[7] = {0,0,1,3,15,60,300};
// status
uint8_t playmode = 0, Step = 0;


uint16_t AD_RES = 0;
	    uint16_t Min_Pulse[4] , Max_Pulse[4] ;
	    float temp[4] ;
	    uint16_t Servo_Pulse[4] ;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */



typedef struct Flags
{
	uint8_t ADCCMPLT;

}flag_t;

flag_t Flg = {0, };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 uint8_t data[20]="!1:TEMP:33#";
 uint8_t stringAck[1]="1";
 uint32_t systicks = 0;
 uint8_t flagLight = 0;

// void HAL_SYSTICK_Callback(void)
// {
//     systicks++;
//     if (systicks >= 1000) // nếu đã đếm đủ 1s
//     {
//         systicks = 0;
//         flagLight = 1; // đặt biến c�? để báo hiệu cho hàm main
//     }
// }

 void task1(){

	 Light=200-(value[0]*200)/4096;
	 sprintf(string, "!1:LIGHT:%d#",Light );
	 HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 100);
 }
 void task2(){
if  (couter!=precouter){precouter=couter;
	 uint32_t string1[20];
	 sprintf(string1, "!1:couter:%d#",couter );
	 HAL_UART_Transmit(&huart1, (uint8_t*)string1, strlen(string1), 100);
  }}
void resentack(){
	status= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	if (status_ack != status){
		 uint32_t string1[20];
		 sprintf(string1, "!1:button2:%d#",status );
		 HAL_UART_Transmit(&huart1, (uint8_t*)string1, strlen(string1), 1);
	}
}
extern void calc_pause()
{
//    readPot();
    tempp = value[3];
    if (tempp < 0) tempp = 0;
    tempp = map(tempp, 0, 4095, 0 ,6);
    verz = Delay[tempp];

}
void Button()
{
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 0)
  {
	  HAL_Delay(20);
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1) // taster losgelassen
    {
      if (Taster == 0)
      {
        Taster = 1;
        previousMillis3 = currentMillis;
        //Serial.print("Status Record "); Serial.println(Taster);
      }
      else if ((Taster == 1) && (currentMillis - previousMillis3 < 250))
      {
        Taster = 2;
        //Serial.println(Taster);
      }
      /*else if ((Taster == 2) && (currentMillis - previousMillis3 < 500))
      {
        Taster = 3;
        Serial.println(Taster);
      }*/
    }
  }

    if ((Taster == 1) && (currentMillis - previousMillis3 > 1000)) // array beschreiben
    {
      arrayStep += 1;
      arrayMax = arrayStep;
      record();
      Taster = 0;
      playmode = 0;


      HAL_Delay(100);

    }
    else if (Taster == 2)
    {
      arrayStep = 0;
      playmode = 1;
      Taster = 0;
      Step = 1;


      HAL_Delay(250);
//      digitalWrite(13, LOW);
    }
    /*if (Taster == 3)
    {
      // ++ arrayStep
      // playmode = 1;
      Taster = 0;
      Serial.println("Clear ");
    }*/
    if (currentMillis - previousMillis3 > 2000) // Taster Status löschen
    {
      Taster = 0;
      //Serial.println("restart ");
    }
}
void record()
{
    joint0[arrayStep] = Servo_Pulse[0];
    joint1[arrayStep] = Servo_Pulse[1];
    joint2[arrayStep] = Servo_Pulse[2];
    joint3[arrayStep] = Servo_Pulse[3];
}
void play_servo()
{
    steps += 1;
    if (steps < stepsMax)
    {
      //time = del*5;// anfahr rampe
      if(steps == 20) time = del*3;          // anfahr rampe up
      else if(steps == 40) time = del*3;
      else if(steps == 80) time = del*2;
      else if(steps == 100) time = del;
      if(steps == stepsMax-200) time = del*2;        // stop rampe down
      else if(steps == stepsMax-80) time = del*2;
      else if(steps == stepsMax-40) time = del*3;
      else if(steps == stepsMax-20) time = del*3;

      Servo_Pulse[0] += dir[0]; // set new pos
      Servo_Pulse[1] += dir[1];
      Servo_Pulse[2] += dir[2];
      Servo_Pulse[3] += dir[3];

      SERVO_RawMove(SERVO_Motor1, Servo_Pulse[0]); // Zange //anschlüsse gemappt!
      SERVO_RawMove(SERVO_Motor2, Servo_Pulse[1]); // Hand
      SERVO_RawMove(SERVO_Motor3, Servo_Pulse[2]); // Schulter
      SERVO_RawMove(SERVO_Motor4, Servo_Pulse[3]); // Ellbogen
    }
    else
    {
      Step = 1; // next step aus array lesen
      steps = 0; // servo zwischenschritte
    }
}
int myabs(int num){
	if (num < 0) {
	    return -num;
	  } else {
	    return num;
	  }
}
void calculate()
{
      // wegstrecken berechnen
      dif[0] = myabs(Servo_Pulse[0]-sol[0]);
      dif[1] = myabs(Servo_Pulse[1]-sol[1]);
      dif[2] = myabs(Servo_Pulse[2]-sol[2]);
      dif[3] = myabs(Servo_Pulse[3]-sol[3]);

      // grösten weg finden
      stepsMax = max(dif[0],dif[1]);
      stepsMax = max(stepsMax,dif[2]);
      stepsMax = max(stepsMax,dif[3]);
      stepsMax=800;
      //Serial.println(stepsMax);

      if (stepsMax < 500) // slow if maxstep < 400
        del = 1;
      else
        del = 1;

       // einzelschritt berechnen (step + pos/neg)
      if (sol[0] < Servo_Pulse[0]) dir[0] = 0-dif[0]/stepsMax; else dir[0] = dif[0]/stepsMax;
      if (sol[1] < Servo_Pulse[1]) dir[1] = 0-dif[1]/stepsMax; else dir[1] = dif[1]/stepsMax;
      if (sol[2] < Servo_Pulse[2]) dir[2] = 0-dif[2]/stepsMax; else dir[2] = dif[2]/stepsMax;
      if (sol[3] < Servo_Pulse[3]) dir[3] = 0-dif[3]/stepsMax; else dir[3] = dif[3]/stepsMax;
        //Serial.println(dir4);

}
int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int max(int a, int b) {
	if (a>b){
	return a;}
	else {
		return b;
	}
}
void Read()
{
    sol[0] = joint0[arrayStep];
    sol[1] = joint1[arrayStep];
    sol[2] = joint2[arrayStep];
    sol[3] = joint3[arrayStep];
}
void play(){
	if(!playmode) // manualy modus
	  {
		control_servo();
	  }
	 else if(playmode){
		 if(status_servo) {
		 if (Step) // next step read from array
		     {if (arrayStep < arrayMax) // we not reach the end from stored data
		      {
		        arrayStep += 1; // next array pos
		      Read(); // from the arrays
		     calculate(); // find biggest travel distance and calculate the other 3 servos (the have to do smaler steps to be finished at same time!)
		        Step = 0;

		      }
		     else // array read finished > start over
		           {     status_servo=0;
		           	   	   couter++;
						 arrayStep = 0; //
//						 calc_pause(); // delay between moves read from potentiometer
						 countverz = 0; // used for the delay
						 while(countverz < 300) // verz = time getting from calc_pause();
						 { // here we do loop and wait until next start over
						   countverz += 1;
//						   calc_pause();

						 }
		           }

		     }
		 else // do the servos!
		     {  //if (currentMicros - previousMicros > time) // speed für servo update/step
		      {
		        previousMicros = currentMicros;
		        play_servo();
		      }
		     }
	 }
	 }
}

void control_servo(){
//	 SERVO_MoveTo(SERVO_Motor1, 0);
//	 SERVO_MoveTo(SERVO_Motor2, 0);
//	 SERVO_MoveTo(SERVO_Motor3, 0);
//  SERVO_MoveTo(SERVO_Motor4, 90);
//  systicks++;
////
//uint16_t	ma[] = { 90, 20, 80, 150 };
//for (uint16_t i=0;i<=ma[0];i++){
//	SERVO_MoveTo(SERVO_Motor1, i);
//	 HAL_Delay(100);
//}
//for (uint16_t i=0;i<=ma[1];i++){
//	SERVO_MoveTo(SERVO_Motor2, i);
//	 HAL_Delay(100);
//}
//for (uint16_t i=0;i<=ma[2];i++){
//	SERVO_MoveTo(SERVO_Motor3, i);
//	 HAL_Delay(100);
//}
//for (uint16_t i=90;i<=ma[3];i++){
//	SERVO_MoveTo(SERVO_Motor4, i);
//	 HAL_Delay(100);
//}


	Servo_Pulse[0] = ((4095-value[0])*temp[0]) + Min_Pulse[0];
	SERVO_RawMove(SERVO_Motor1, Servo_Pulse[0]);


	Servo_Pulse[1] = (uint16_t)((4095-value[1])*temp[1]) + Min_Pulse[1];
	SERVO_RawMove(SERVO_Motor2, Servo_Pulse[1]);

	Servo_Pulse[2] = (uint16_t)((4095-value[2])*temp[2]) + Min_Pulse[2];
	SERVO_RawMove(SERVO_Motor3, Servo_Pulse[2]);

	Servo_Pulse[3] = (uint16_t)((4095-value[3])*temp[3]) + Min_Pulse[3];
	SERVO_RawMove(SERVO_Motor4, Servo_Pulse[3]);

}
//	        SERVO_MoveTo(SERVO_Motor1, 0);

//	               SERVO_MoveTo(SERVO_Motor3, 0);
//	               SERVO_MoveTo(SERVO_Motor4, 0);
//	               HAL_Delay(1500);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    //start adc


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SERVO_Init(SERVO_Motor1);
  SERVO_Init(SERVO_Motor2);
  SERVO_Init(SERVO_Motor3);
  SERVO_Init(SERVO_Motor4);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
//  HAL_ADC_Start_DMA(&hadc1, &var, 1);
  HAL_ADC_Start_DMA(&hadc1, value, 4);
  HAL_UART_Receive_DMA(&huart1, &rx, 1);
  HAL_TIM_Base_Start_IT(&htim2);

  Min_Pulse[0] = SERVO_Get_MinPulse(SERVO_Motor1);
  Max_Pulse[0] = SERVO_Get_MaxPulse(SERVO_Motor1);
  temp[0] = ((Max_Pulse[0]-Min_Pulse[0])/4096.0);

  Min_Pulse[1] = SERVO_Get_MinPulse(SERVO_Motor2);
  Max_Pulse[1] = SERVO_Get_MaxPulse(SERVO_Motor2);
  temp[1] = ((Max_Pulse[1]-Min_Pulse[1])/4096.0);

  Min_Pulse[2] = SERVO_Get_MinPulse(SERVO_Motor3);
  Max_Pulse[2] = SERVO_Get_MaxPulse(SERVO_Motor3);
  temp[2] = ((Max_Pulse[2]-Min_Pulse[2])/4096.0);

  Min_Pulse[3] = SERVO_Get_MinPulse(SERVO_Motor4);
  Max_Pulse[3] = SERVO_Get_MaxPulse(SERVO_Motor4);
  temp[3] = ((Max_Pulse[3]-Min_Pulse[3])/4096.0);

  SCH_Add_Task(task1, 1000, 12000);
  SCH_Add_Task(task2, 1000, 500);
  SCH_Add_Task(resentack, 6000, 3300);
  SCH_Add_Task(Button, 10,1);
  SCH_Add_Task(play, 12,1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)

  {
	  SCH_Dispatch_Tasks();
//	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1) // taster losgelassen
//	      { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
//	      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);}
//	  else {
//		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);
//	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  HAL_ADC_Start(&hadc1);
//	  var = HAL_ADC_GetValue(&hadc1);
//	  Temp =  (1475 - ((2475 * var)) / 4096)/12;
//	  HAL_ADC_Stop(&hadc1);
//	  if (flagLight==1)
//	   { flagLight = 0;
//		   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
//		   if (rx_index==0){ rx_index=3;
//		   Light=200-(Adc.Raw[1]*200)/4096;
//		   	sprintf(string, "!1:LIGHT:%d#",Light );
//		   	HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 2);
//		   };
//		   rx_index--;
//	   }
//		   if (Flg.ADCCMPLT) /* Conversion completed, do calculations */
//		   		{
//
//		   			/* Temperature Sensor ADC-value, Reference Voltage ADC-value (if use) */
////		   			Adc.IntSensTmp = TMPSENSOR_getTemperature(Adc.Raw[1], Adc.Raw[0]);
////		   			var = 200-(Adc.Raw[1]*200)/4096;
//
//
//		   			Flg.ADCCMPLT = 0; /* Nullify flag */
//		   		}
//		   HAL_UART_Transmit(&huart1, data, sizeof(data), 2);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{   if (GPIO_Pin == GPIO_PIN_13){
	status= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	if (status==0){
			status=1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart1,"!1:button2:1#" , 12, 2);
	}else{
			status=0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_UART_Transmit(&huart1,"!1:button2:0#" , 12, 2);
}
	}}
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{ if (hadc ->Instance == hadc1.Instance){
//	char string[20];
//		Light=200-(Adc.Raw[1]*200)/4096;
//		sprintf(string, "!1:LIGHT:%d#",Light );
//		HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 2);
//}
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {if ( huart -> Instance == huart1.Instance){
//	 HAL_UART_Receive_IT(&huart1, data, 2);}
//
//
// }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{if (huart->Instance == USART1) {
	switch (rx){
	case '1':  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart1,"10" , 2, 2);

				break;
	case '2':  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart1,"11" , 2, 2);
				break;
	case '3': 		 status_servo=0;
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_UART_Transmit(&huart1,"20" , 2, 2);
					break;
	case '4':  		status_servo=1;
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart1,"21" , 2, 2);
					break;
	case '5':  status_ack=0;
				break;
	case '6':  status_ack=1;
				break;
	case '7':  status_servo=1;
					break;
	case '8':  status_servo=0;
					break;
	}

}
}

int cou=0;
void  HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2){
		SCH_Update();
		currentMillis++;
		currentMicros++;
	cou++;




}}

//    if (huart->Instance == USART1) // nếu là UART1
//    {
//        if (rx_data == '#') // nếu nhận được ký tự kết thúc dòng
//        {
//            rx_buffer[rx_index] = '\0'; // thêm ký tự NULL vào cuối bộ đệm
//            // xử lý dữ liệu nhận được ở đây
//            if (rx_buffer[0]=='!'){
//            	char x1[7]= {'E','D',':','T','E','M','P'}; uint8_t a,b=0;
//            	           for ( b=3;b<6;b++){
//            	                if(rx_buffer[b]!=x1[b]) goto ab;
//            	                else { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//									a=1; goto ab;
//								}
//            }}
//          ab:  // reset bộ đệm nhận UART
//            rx_index = 0;
//            memset(rx_buffer, 0, RX_BUFFER_SIZE);
//            HAL_UART_AbortReceive(huart);
//            HAL_UART_Receive_DMA(huart, &rx_data, 1);
//        }
//        else
//        {
//            rx_buffer[rx_index] = rx_data;
//            rx_index++;
//            HAL_UART_Receive_DMA(huart, &rx_data, 1);
//        }
//    }






/* USER CODE END 4 */

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
