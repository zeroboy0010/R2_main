/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_NRF24.h"
#include "PID.h"
#include "omni_wheel.h"
#include "AccelStepper.h"
#include "micros.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PIDController MPID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define MKp 7.92
#define MKi 136.48
#define MKd 0.0

float Motor1_speed = 0;
float Motor2_speed = 0;
float Motor3_speed = 0;
float Motor4_speed = 0;
float V1 = 0; // target speed of motor1
float V2 = 0; // target speed of motor2
float V3 = 0; // target speed of motor3
float V4 = 0; // target speed of motor4
float pwm_M1 = 0;
float pwm_M2 = 0;
float pwm_M3 = 0;
float pwm_M4 = 0;

float M_down, M_up;
float pwm_M = 1000;
float pwm_cnt = 0;

// to get data from nrf
float vx1, vy1, omega1;
float vx, vy, omega;

#define maxV 500.0
#define maxOmega 6.50

uint64_t RxpipeAddrs = 0x23456744BB;
uint8_t myRxData[12];

// float pwm1, pwm2, pwm3, pwm4;
//  button
uint8_t BTN = 0;

// stepper
Stepper_t stepper_upper;
Stepper_t stepper_under;

uint8_t limit_under = 0;
uint8_t homing_stepper_under = 0;
// stepper 2

uint8_t limit_upper = 0;
uint8_t homing_stepper_upper = 0;
uint8_t moveTo_mode = 0;

// button for manual mode
uint8_t B_xi = 0;
uint8_t B_xo = 0;
uint8_t B_x_0 = 0;

uint8_t B_yi = 0;
uint8_t B_yo = 0;
uint32_t new_count_S;
old_count_S; // for stoping robot when loss signal
uint8_t joy_Ry;
// button for auto mode
uint8_t switch_mode = 0;
uint8_t Button_logari_1 = 0;
uint8_t Button_logari_2 = 0;
uint8_t Button_logari_3 = 0;
uint8_t Button_logari_4 = 0;
uint8_t Button_logari_5 = 0;
uint8_t Button_logari_start = 0;
uint8_t Button_logari_release = 0;
uint8_t Button_logari_reset = 0;

long Data_x[7] = {0,29000,46000,64000,78000,92000};
int Data_z[7] = {0,0,0,0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output)
{

	return (float)((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}
void reset_data(void)
{
	myRxData[0] = 128; // x1
	myRxData[1] = 128; // y1
	myRxData[2] = 128; // x2
	myRxData[3] = 128; // y2
	myRxData[7] = 0;   // all button
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	// PID init--------------------------------
	PID_Init(&MPID, 4);
	MPID.T = 0.01; // T = 10ms
	MPID.limMax = 1000;
	MPID.limMin = -1000;
	MPID.limMaxInt = 1000;
	MPID.limMinInt = -1000;
	MPID.tau = 0; // for Kd
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	DWT_Init();
	InitStepper(&stepper_upper, step2_Pin, step2_GPIO_Port, dir2_Pin, dir2_GPIO_Port, 0);
	setAcceleration(&stepper_upper, 500);
	setMaxSpeed(&stepper_upper, 1000);
	moveTo(&stepper_upper, -100000);

	InitStepper(&stepper_under, step1_Pin, step1_GPIO_Port, dir1_Pin, dir1_GPIO_Port, 0);
	setMaxSpeed(&stepper_under, 300);
	setAcceleration(&stepper_under, 100);
	moveTo(&stepper_under, -20000);

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM7_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	NRF24_begin(GPIOD, GPIO_PIN_7, GPIO_PIN_6, hspi1);
	nrf24_DebugUART_Init(huart1);

	printRadioSettings();
	NRF24_setAutoAck(true);
	NRF24_setChannel(68);
	NRF24_setPayloadSize(32);
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_startListening();
	reset_data();

	// timer
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// map that data
		// limit switch
		limit_upper = HAL_GPIO_ReadPin(limit1_GPIO_Port, limit1_Pin);
		limit_under = HAL_GPIO_ReadPin(limit2_GPIO_Port, limit2_Pin);

		if (homing_stepper_under == 1 && homing_stepper_upper == 1)
		{
			if (NRF24_available())
			{
				NRF24_read(myRxData, 32);
				vx1 = myRxData[0];
				vy1 = myRxData[1];
				omega1 = myRxData[2];
				joy_Ry = myRxData[3];
				switch_mode = myRxData[4];
				BTN = myRxData[5];
				new_count_S = myRxData[6];
				while (hal_gettick() - x >= 100) // 100ms
				{
					if (new_count_S == old_count_S)
						lose_signal = 1;
					else
						lose_signal = 0;
					old_count_S = new_count_S;
				}
			}
			if (vx1 != 0 && vy1 != 0 && omega1 != 0)
			{
				vx = map(vx1, 0, 256, -1.0 * maxV, maxV);
				vy = map(vy1, 0, 256, -1.0 * maxV, maxV);
				omega = map(omega1, 0, 256, maxOmega, -1.0 * maxOmega);
				// using button
				if (switch_mode == 0)
				{
					B_yi = BTN >> 1 & 1;
					B_yo = BTN >> 0 & 1;
					B_xi = BTN >> 4 & 1;
					B_xo = BTN >> 6 & 1;
					B_x_0 = BTN >> 5 & 1;
					M_up = BTN >> 2 & 1;
					M_down = BTN >> 7 & 1;
				}
				else if (switch_mode == 1)
				{
					// Button_logari_1 = BTN >> 0 & 1;
					// Button_logari_2 = BTN >> 1 & 1;
					// Button_logari_3 = BTN >> 2 & 1;
					// Button_logari_4 = BTN >> 3 & 1;
					// Button_logari_5 = BTN >> 4 & 1;
					Button_logari_start = BTN >> 5 & 1;
					Button_logari_release = BTN >> 6 & 1;
					Button_logari_reset = BTN >> 7 & 1;
				}
			}
		}

		// stepper upper --------------------------------------------------------------

		if (limit_upper == 1 && homing_stepper_upper == 0)
		{
			stop(&stepper_upper);
			stepper_upper._speed = 0;
			setCurrentPosition(&stepper_upper, 0);
			stepper_upper._targetPos = 0;
			// moveTo(&stepper_upper, 0);
			homing_stepper_upper = 1;
			setAcceleration(&stepper_upper, 2000);
			setMaxSpeed(&stepper_upper, 8000);
		}

		if (distanceToGo(&stepper_upper) != 0)
		{
			run(&stepper_upper);
		}
		// stepper-----------------------------------------------------------

		if (limit_under == 1 && homing_stepper_under == 0)
		{
			stop(&stepper_under);
			stepper_under._speed = 0;
			setCurrentPosition(&stepper_under, -100);
			stepper_under._targetPos = -100;
			moveTo(&stepper_under, 0);
			homing_stepper_under = 1;
			setAcceleration(&stepper_under, 100);
			setMaxSpeed(&stepper_under, 300);
		}

		if (distanceToGo(&stepper_under) != 0)
		{
			run(&stepper_under);
		}
		// mode
		if (switch_mode == 0)	// manual mode
		{
			if (homing_stepper_upper == 1)
			{
				// moveTo(&stepper_upper, x);
				if (B_xi == 0 && B_xo == 1 && B_x_0 == 0)
				{
					move(&stepper_upper, -50000);
					moveTo_mode = 0;
				}
				else if (B_xi == 1 && B_xo == 0 && B_x_0 == 0)
				{
					move(&stepper_upper, 50000);
					moveTo_mode = 0;
				}
				else if (B_xi == 0 && B_xo == 0 && B_x_0 == 1)
				{
					moveTo(&stepper_upper, 500);
					moveTo_mode = 1;
				}
				else
				{
					if (moveTo_mode == 0)
					{
						stop(&stepper_upper);
						stepper_upper._speed = 0;
						setCurrentPosition(&stepper_upper, currentPosition(&stepper_upper));
						stepper_upper._targetPos = currentPosition(&stepper_upper);
					}
				}
			}
			//------------------------------------------------------
						if (homing_stepper_under == 1)
			{
				if (B_yi == 0 && B_yo == 1)
				{
					moveTo(&stepper_under, 12000);
				}
				else if (B_yi == 1 && B_yo == 0)
				{
					moveTo(&stepper_under, 0);
				}
			}
		}
		else if (switch_mode == 1)	// automode
		{
			switch (BTN) {
				case 1 : 

					break;
				case 2 :

					break;
			}
		}
		//	BTN signal LED---------------------------------------------------------------------------
		if (BTN > 0)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
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
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == M1_CHA_Pin || M1_CHB_Pin)
	{ // ENCODER Motor 1
		nowA[0] = HAL_GPIO_ReadPin(M1_CHA_GPIO_Port, M1_CHA_Pin);
		nowB[0] = HAL_GPIO_ReadPin(M1_CHB_GPIO_Port, M1_CHB_Pin);
		Enc_count[0] = encoder(0);
	}
	if (GPIO_Pin == M2_CHA_Pin || M2_CHB_Pin)
	{ // ENCODER Motor 1
		nowA[1] = HAL_GPIO_ReadPin(M2_CHA_GPIO_Port, M2_CHA_Pin);
		nowB[1] = HAL_GPIO_ReadPin(M2_CHB_GPIO_Port, M2_CHB_Pin);
		Enc_count[1] = encoder(1);
	}
	if (GPIO_Pin == M3_CHA_Pin || M3_CHB_Pin)
	{ // ENCODER Motor 1
		nowA[2] = HAL_GPIO_ReadPin(M3_CHA_GPIO_Port, M3_CHA_Pin);
		nowB[2] = HAL_GPIO_ReadPin(M3_CHB_GPIO_Port, M3_CHB_Pin);
		Enc_count[2] = encoder(2);
	}
	if (GPIO_Pin == M4_CHA_Pin || M4_CHB_Pin)
	{ // ENCODER Motor 1
		nowA[3] = HAL_GPIO_ReadPin(M4_CHA_GPIO_Port, M4_CHA_Pin);
		nowB[3] = HAL_GPIO_ReadPin(M4_CHB_GPIO_Port, M4_CHB_Pin);
		Enc_count[3] = encoder(3);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7)
	{
		// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		if ((M_down == 1 && M_up == 0) || (M_down == 0 && M_up == 1))
		{
			if (pwm_cnt <= pwm_M)
			{
				pwm_cnt += 100;
			}
		}
		else
		{
			pwm_cnt = 0;
		}

		// wheel
		V1 = wheel1(vx, vy, omega);
		V2 = wheel2(vx, vy, omega);
		V3 = wheel3(vx, vy, omega);
		V4 = wheel4(vx, vy, omega);
		// PID
		pwm_M1 = PID(&MPID, V1, Motor1_speed, MKp, MKi, MKd, Motor1);
		pwm_M2 = PID(&MPID, V2, Motor2_speed, MKp, MKi, MKd, Motor2);
		pwm_M3 = PID(&MPID, V3, Motor3_speed, MKp, MKi, MKd, Motor3);
		pwm_M4 = PID(&MPID, V4, Motor4_speed, MKp, MKi, MKd, Motor4);
		// feedback speed
		Motor1_speed = Motors_RPS(Motor1, 10, 912);
		Motor2_speed = Motors_RPS(Motor2, 10, 912);
		Motor3_speed = Motors_RPS(Motor3, 10, 912);
		Motor4_speed = Motors_RPS(Motor4, 10, 912);

		// pwm ---------------------------------------------------------------------------------
		if (pwm_M1 > 0)
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = pwm_M1;
		}
		else if (pwm_M1 < 0)
		{
			TIM1->CCR1 = -1.0 * pwm_M1;
			TIM1->CCR2 = 0;
		}
		else
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
		}
		// motor 2
		if (pwm_M2 > 0)
		{
			TIM1->CCR3 = 0;
			TIM1->CCR4 = pwm_M2;
		}
		else if (pwm_M2 < 0)
		{
			TIM1->CCR3 = -1.0 * pwm_M2;
			TIM1->CCR4 = 0;
		}
		else
		{
			TIM1->CCR3 = 0;
			TIM1->CCR4 = 0;
		}
		// motor 3
		if (pwm_M3 > 0)
		{
			TIM3->CCR1 = pwm_M3;
			TIM3->CCR2 = 0;
		}
		else if (pwm_M3 < 0)
		{
			TIM3->CCR1 = 0;
			TIM3->CCR2 = -1.0 * pwm_M3;
		}
		else
		{
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
		}
		// motor 4
		if (pwm_M4 > 0)
		{
			TIM3->CCR3 = pwm_M4;
			TIM3->CCR4 = 0;
		}
		else if (pwm_M4 < 0)
		{
			TIM3->CCR3 = 0;
			TIM3->CCR4 = -1.0 * pwm_M4;
		}
		else
		{
			TIM3->CCR3 = 0;
			TIM3->CCR4 = 0;
		}

		// up and down motor -----------------------------------------------
		if (M_down == 1 && M_up == 0)
		{
			TIM4->CCR1 = pwm_cnt;
			TIM4->CCR2 = 0;
		}
		else if (M_down == 0 && M_up == 1)
		{
			TIM4->CCR1 = 0;
			TIM4->CCR2 = pwm_cnt;
		}
		else
		{
			TIM4->CCR1 = 0;
			TIM4->CCR2 = 0;
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
