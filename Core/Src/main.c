/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "ssd1306/ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 3

#define PWM_BUFFER_SIZE 32
#define PWM_HALFBUFFER_SIZE 16
#define DITHER_RES 8

#define COR_ALPHA_BUFFER_SIZE 3
#define COR_VMES_BUFFER_SIZE 3
#define COR_VMI_BUFFER_SIZE 3
#define COR_VREF_BUFFER_SIZE 3
#define COR_VCONTRE_BUFFER_SIZE 3
#define COR_VREFPRIME_BUFFER_SIZE 3
#define COR_EPSILON_BUFFER_SIZE 3

#define NB_ECRAN 3


//float Hz[] = { 0, 35.28, 18.32, 0.9834, 0.2503 };
#define Hz0 0f
#define Hz1 35.28f
#define Hz2 18.32f
#define Hz3 0.9834f
#define Hz4 0.2503f


//float Gz[] = { 0.09683, 0.07675, 0, -1.326, 0.4996 };
#define Gz0 0.09683f
#define Gz1 0.07675f
#define Gz2 0f
#define Gz3 -1.326f
#define Gz4 0.4996f


//float Fz[] = { 1.284, 0.5245, 0, 0.6743, 0.1345 };
#define Fz0 1.284f
#define Fz1 0.5245f
#define Fz2 0f
#define Fz3 0.6743f
#define Fz4 0.1345f

//float Cz[] = { 0.01865, 0.01834, 0.004668, 0, 0 };
#define Cz0 0.01865f
#define Cz1 0.01834f
#define Cz2 0.004668f
#define Cz3 0f
#define Cz4 0f


enum rotary_direction {
	DIRECTION_NONE, DIRECTION_CW, DIRECTION_CCW, NUM_OF_DIRECTIONS
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t rotary_state;
static uint8_t rotary_counter = 0;
static int8_t rotary_direction = DIRECTION_NONE;

static uint8_t ecran = 0;

static bool btn = true;
static bool prev_btn = true;

uint32_t voltage_in_ADC = 0;
uint32_t voltage_out_ADC = 0;
uint32_t current_ADC = 0;

float voltage_in = 0;
float voltage_out = 0;
float current = 0;
float voltage_ref = 0;

uint16_t adc_buf[ADC_BUF_LEN];

/*float corAlpha[COR_ALPHA_BUFFER_SIZE];
float corVref[COR_VMES_BUFFER_SIZE];
float corVmi[COR_VMI_BUFFER_SIZE];
float corVmes[COR_VREF_BUFFER_SIZE];
float corVcontre[COR_VCONTRE_BUFFER_SIZE];
float corVrefprime[COR_VREFPRIME_BUFFER_SIZE];
float corEpsilon[COR_EPSILON_BUFFER_SIZE];*/


float corAlpha = 0;
float corAlpha_1 = 0;
float corAlpha_2 = 0;

float corVref = 0;
float corVref_1 = 0;
float corVref_2= 0;

float corVmi = 0;
float corVmi_1 = 0;
float corVmi_2 = 0;

float corVmes = 0;
float corVmes_1 = 0;
float corVmes_2 = 0;

float corVcontre = 0;
float corVcontre_1 = 0;
float corVcontre_2 = 0;

float corVrefprime = 0;
float corVrefprime_1 = 0;
float corVrefprime_2 = 0;

float corEpsilon = 0;
float corEpsilon_1 = 0;
float corEpsilon_2 = 0;




//float Hz[] = { 0, 35.28, 18.32, 0.9834, 0.2503 };
#define Hz0 0f
#define Hz1 35.28f
#define Hz2 18.32f
#define Hz3 0.9834f
#define Hz4 0.2503f


//float Gz[] = { 0.09683, 0.07675, 0, -1.326, 0.4996 };
#define Gz0 0.09683f
#define Gz1 0.07675f
#define Gz2 0f
#define Gz3 -1.326f
#define Gz4 0.4996f


//float Fz[] = { 1.284, 0.5245, 0, 0.6743, 0.1345 };
#define Fz0 1.284f
#define Fz1 0.5245f
#define Fz2 0f
#define Fz3 0.6743f
#define Fz4 0.1345f

//float Cz[] = { 0.01865, 0.01834, 0.004668, 0, 0 };
#define Cz0 0.01865f
#define Cz1 0.01834f
#define Cz2 0.004668f
#define Cz3 0f
#define Cz4 0f


// Tramage (dithering)
uint16_t pwm_buffer[PWM_BUFFER_SIZE];
uint8_t pwm_pulse = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Human interface
void Update_Display(void);

// USB Power Delivery requests
uint8_t Read_USB_PDO(void);
void Test_STUSB_I2C(void);
void Set_STUSB_Available_Profiles(uint8_t num_profiles);

// Voltage controller
int32_t Compute_control_input(float Vref, float Vmes);
void TransferComplete(DMA_HandleTypeDef *hdma);
void TransferHalfComplete(DMA_HandleTypeDef *hdma);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM22_Init();

	/* USER CODE BEGIN 2 */

	// Initialize screen
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc_buf, ADC_BUF_LEN);

	// Start PWM timer
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*) pwm_buffer,
	PWM_BUFFER_SIZE);

	// Start main timer
	htim2.hdma[TIM_DMA_ID_CC2]->XferCpltCallback = TransferComplete;
	htim2.hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = TransferHalfComplete;
	HAL_TIM_Base_Start_IT(&htim22);

	// Switch off user LED
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

	//Enable MP8040 pin PA10 (activation V sortie )
	HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//MàJ bouton
		prev_btn = btn;
		btn = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);

		// 1.1 Menu et navigation
		Update_Display();

		/* Reading PDOs TOR inputs
		 * Code :
		 *  PDO = 0x000ABCD
		 *  A = PDO2
		 *  B = PDO3
		 *  C = PDO4
		 *  D = PDO5
		 */
		uint8_t PDO2 = HAL_GPIO_ReadPin(PDO2_GPIO_Port, PDO2_Pin);
		uint8_t PDO3 = HAL_GPIO_ReadPin(PDO3_GPIO_Port, PDO3_Pin);
		uint8_t PDO4 = HAL_GPIO_ReadPin(PDO4_GPIO_Port, PDO4_Pin);
		uint8_t PDO5 = HAL_GPIO_ReadPin(PDO5_GPIO_Port, PDO5_Pin);
		uint8_t PDO = (PDO5 << 3) | (PDO4 << 2) | (PDO3 << 1) | PDO2;

		/* Decoding PDOs
		 *  PDO2 = 5V
		 *  PDO3 = 9V
		 *  PDO4 = 12V
		 *  PDO5 = 15V
		 */
		switch (PDO) {
		case 0b00001110:
			voltage_ref = 5;
			break;
		case 0b00001101:
			voltage_ref = 9;
			break;
		case 0b00001011:
			voltage_ref = 12;
			break;
		case 0b00000111:
			voltage_ref = 15;
			break;
		}

		// TODO
		// 4.1 Lecture PDO et asservissement

		// TODO
		// 2.3 Calcul énergie et Puissance

		// Delay
		HAL_Delay(25);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/*** IHM ***/

/*** USB Power Delivery Objects ***/

void Test_STUSB_I2C(void) {
	char msg[60];
	uint8_t id;
	// Device I2C address : 0x28
	// Device ID register : 0x2F
	int ret = HAL_I2C_Mem_Read(&hi2c1, (0x28 << 1), 0x70, 1, &id, 1,
			0xFFFFFFFF);
	if (ret == HAL_OK) {
		sprintf(msg, "STUSB I2C test success\r\n");
	} else {
		sprintf(msg, "STUSB I2C test error %d\r\n", ret);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

void Set_STUSB_Available_Profiles(uint8_t num_profiles) {
	// TODO
	// 4.3 Programmation PDOs

	/** Documentation for write number of available profiles
	 * I2C communication:
	 * Use HAL_I2C_Mem_Read and HAL_I2C_Mem_Write functions to directly read/write into register
	 * WRN: Device address is 0x28 but should be passed shifted (0x28 << 1) to those functions
	 * Cf. Test_STUSB_I2C function
	 *
	 * Register 0x70 DPM_PDO_NUMB
	 * bit [7:5] DPM_SRC_PDO_NUMB <--- Number of source profiles available
	 * bit [4:3] reserved
	 * bis [2:0] DPM_SNK_PDO_NUMB
	 *
	 * Procedure :
	 * 1) Set device into reset state : write 0x01 to register 0x23
	 * 2) Write new value to regsiter 0x70, for ex. 0xA3 to set all 5 profiles available (1010001)
	 * Or write for example 0x23 to set only one profile available (00100011)
	 * 3) Set device back to normal mode : write 0x04 to register 0x23
	 */
}

/*** Voltage controller ***/

int32_t Compute_control_input(float Vref, float Vmes) {
	// TODO
	// 3.2 Régulation

	// Shifting
	corAlpha_2 = corAlpha_1;
	corAlpha_1 = corAlpha;

	corVmes_2 = corVmes_1;
	corVmes_1 = corVmes;

	corVref_2 = corVref_1;
	corVref_1 = corVref;

	corEpsilon_2 = corEpsilon_1;
	corEpsilon_1 = corEpsilon;

	corVmi_2 = corVmi_1;
	corVmi_1 = corVmi;

	corVcontre_2 = corVcontre_1;
	corVcontre_1 = corVcontre;

	corVrefprime_2 = corVrefprime_1;
	corVrefprime_1 = corVrefprime;


	// Storing new voltage values
	corVref = Vref;
	corVmes = Vmes;

	// Computing alpha
	uint32_t pwm_result = 0;

	//VERIFIER QUE C'EST BIEN MAINTENANT QU'ON DOIT FAIRE CA !

	//corAlpha = Vref/10;//   !!! pk

	corVmi = Hz1 * corAlpha_1 + Hz2 * corAlpha_2 - Hz3 * corVmi_1
			- Hz4 * corVmi_2;

	corVcontre = Fz0 * (corVmes - corVmi)
			+ Fz1 * (corVmes_1 - corVmi_1) - Fz3 * corVcontre_1
			- Fz4 * corVcontre_2;

	corVrefprime = Gz0 * corVref + Gz1 * corVref_1
			- Gz3 * corVrefprime_1 - Gz4 * corVrefprime_2;

	corAlpha = Cz0 * (corVrefprime - corVcontre)
			+ Cz1 * (corVrefprime_1 - corVcontre_1)
			+ Cz2 * (corVrefprime_2 - corVcontre_2);

	pwm_result = (uint32_t)(corAlpha*255.0);
	return pwm_result;
}

/*******************************************************************
 * INTERRUPTS
 *******************************************************************/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	// DONE
	// 2.1 Conversion Analogique-Numérique
	//Acquire ADC values
	voltage_in_ADC = adc_buf[1];
	voltage_out_ADC = adc_buf[0];
	current_ADC = adc_buf[2];
	// 2.2 Etalonage
	voltage_in = ((float) voltage_in_ADC - 62) / 134.4;
	voltage_out = ((float) voltage_out_ADC - 62) / 134.4;
	current = current_ADC;

	// TODO
	// 3.1 Gestion PWM
	uint32_t controle_output = Compute_control_input(voltage_ref, voltage_out);
	pwm_pulse = (uint8_t)controle_output;

	// TODO
	// 4.2 Protection

}

void TransferComplete(DMA_HandleTypeDef *hdma) {
	//TODO
	// 3.3 Filtrage sortie

	for (int i = 0; i < PWM_HALFBUFFER_SIZE; i++) {
		pwm_buffer[i + PWM_HALFBUFFER_SIZE] = pwm_pulse;
	}
}

void TransferHalfComplete(DMA_HandleTypeDef *hdma) {
	//TODO
	// 3.3 Filtrage sortie

	for (int i = 0; i < PWM_HALFBUFFER_SIZE; i++) {
		pwm_buffer[i] = pwm_pulse;

	}
}

void Rotary_Encoder_Interrupt_Handler(void) {
	static int8_t rotary_buffer = 0;
	/* Check for rotary encoder turned clockwise or counter-clockwise */
	uint8_t rotary_new = HAL_GPIO_ReadPin(ROT_CHA_GPIO_Port, ROT_CHA_Pin) << 1;
	rotary_new += HAL_GPIO_ReadPin(ROT_CHB_GPIO_Port, ROT_CHB_Pin);
	if (rotary_new != rotary_state) {
		if (((rotary_state == 0b00) && (rotary_new == 0b10))
				|| ((rotary_state == 0b10) && (rotary_new == 0b11))
				|| ((rotary_state == 0b11) && (rotary_new == 0b01))
				|| ((rotary_state == 0b01) && (rotary_new == 0b00))) {
			rotary_buffer++;
		}
		if (((rotary_state == 0b00) && (rotary_new == 0b01))
				|| ((rotary_state == 0b01) && (rotary_new == 0b11))
				|| ((rotary_state == 0b11) && (rotary_new == 0b10))
				|| ((rotary_state == 0b10) && (rotary_new == 0b00))) {
			rotary_buffer--;
		}
		rotary_state = rotary_new;
		/* Filter rotation due to high number of positions */
		if (rotary_buffer > 2) {
			rotary_counter++;
			if (ecran < (NB_ECRAN - 1) * 10) {
				ecran = (ecran / 10 + 1) * 10;
			} else {
				ecran = 0;
			}

			rotary_direction = DIRECTION_CW;
			rotary_buffer = 0;
		}
		if (rotary_buffer < -2) {
			rotary_counter--;
			rotary_direction = DIRECTION_CCW;
			rotary_buffer = 0;
			if (ecran > 0) {
				ecran = (ecran / 10 - 1) * 10;
			} else {
				ecran = (NB_ECRAN - 1) * 10;
			}
		}

	}
}

void Update_Display(void) {
	//Préparation affichages
	char line1_str[20] = { 0 };
	char line2_str[20] = { 0 };

	//State machine (Display)
	ssd1306_Fill(Black);
	char title_str[20] = { 0 };
	switch (ecran) {
	case 0:
		sprintf(title_str, "Page 1 - ADC");
		sprintf(line1_str, "Vi:%04u   Vo:%04u", (uint16_t) voltage_in_ADC,
				(uint16_t) voltage_out_ADC);
		sprintf(line2_str, "Io:%04u", (uint16_t) current_ADC);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(title_str, Font_7x10, White);
		ssd1306_SetCursor(0, 11);
		ssd1306_WriteString(line1_str, Font_7x10, White);
		ssd1306_SetCursor(0, 22);
		ssd1306_WriteString(line2_str, Font_7x10, White);
		ssd1306_UpdateScreen();
		break;
	case 10:
		sprintf(title_str, "Page 2-1 ADC conv");
		sprintf(line1_str, "Vi:%.2f (V)", (float) voltage_in);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(title_str, Font_7x10, White);
		ssd1306_SetCursor(0, 11);
		ssd1306_WriteString(line1_str, Font_7x10, White);
		ssd1306_UpdateScreen();
		break;
	case 11:
		sprintf(title_str, "Page 2-2 ADC conv");
		sprintf(line1_str, "Vo:%.2f (V)", (float) voltage_out);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(title_str, Font_7x10, White);
		ssd1306_SetCursor(0, 11);
		ssd1306_WriteString(line1_str, Font_7x10, White);
		ssd1306_UpdateScreen();
		break;
	case 12:
		sprintf(title_str, "Page 2-3 ADC conv");
		sprintf(line1_str, "Io:%04u (A)", (uint16_t) current);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(title_str, Font_7x10, White);
		ssd1306_SetCursor(0, 11);
		ssd1306_WriteString(line1_str, Font_7x10, White);
		ssd1306_UpdateScreen();
		break;
	case 20:
		sprintf(title_str, "Page 3");
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(title_str, Font_7x10, White);
		ssd1306_SetCursor(0, 11);
		ssd1306_WriteString(line2_str, Font_7x10, White);
		ssd1306_UpdateScreen();
		break;
	}

	//State machine actions
	switch (ecran) {
	case 10:
		if (prev_btn < btn) {
			ecran++;
		}
		break;
	case 11:
		if (prev_btn < btn) {
			ecran++;
		}
		break;
	case 12:
		if (prev_btn < btn) {
			ecran = 10;
		}
		break;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
