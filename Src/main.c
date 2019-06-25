
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "phone_no.h"
//add your phone number like this:
//#define PHONE_NO "+380123456789"

#include "UART_Buffer.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define AT_PROTOCOL_DELAY 200
#define AT_PROTOCOL_RETRY 6
#define AT_ANSWER_LENGTH_MAX 16
#define SLEEP_ENABLED
#define SEND_SMS

volatile uint32_t flags_reg = 0;
enum {
	SMS_SENDING_FAILED,
	SEND_USB_POW_SMS,
	PIR_SIGNAL_DETECTED,
	ARMED_STATE,
	USR_BTN_PRESSED,
	USB_VBUS_TOGGLED
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t AT_Read_answer(uint8_t* result){
	uint8_t UART_byte_buf, AT_answer[AT_ANSWER_LENGTH_MAX], i=0;

	do{
		if (UART_buf_read(&UART_byte_buf) == NOTHING_TO_READ)
			return 0;
	}while(UART_byte_buf != 13u); //13u stands for \r or <CR> symbol

	while(UART_buf_read(&AT_answer[i]) == READ_OK){
		if((AT_answer[i] == 13u) && i>0)
			break;
		i++;
		if(i == AT_ANSWER_LENGTH_MAX)
			return 0;
	}

	uint8_t drop_symbols = 0;
	while ((AT_answer[drop_symbols] == 10u) ||(AT_answer[drop_symbols] == 13u))
		drop_symbols++;

	uint8_t command_len = i-drop_symbols; //to remove \r\n before the command
	strncpy(result, AT_answer+drop_symbols, command_len);
	result[command_len] = 0u; // because *result array may not be initialized with zeroes
	return command_len;
}

uint8_t AT_expected_received_once(uint8_t* expected_command){
	uint8_t AT_answer[AT_ANSWER_LENGTH_MAX];

	if(AT_Read_answer(AT_answer))
		if(strcmp(AT_answer, expected_command) == 0)
			return 1;

	return 0;
}

uint8_t AT_expected_received(uint8_t* expected_command){
	uint8_t counter = 0;

	do{
		HAL_Delay(AT_PROTOCOL_DELAY);
		if (counter++ >= AT_PROTOCOL_RETRY){
			return 0;
		}
	}while(!AT_expected_received_once(expected_command));
	return 1;
}

uint8_t send_SMS(uint8_t* text, uint8_t length){
#ifdef SEND_SMS
	uint8_t command0[] = "AT+CGREG?\r";
	uint8_t command1[] = "AT+CMGF=1\r";
	uint8_t command2[] = "AT+CMGS=\""PHONE_NO"\"\r";

	HAL_GPIO_WritePin(GPIOA, GSM_POW_EN_Pin, GPIO_PIN_SET); // Enable power supply for GSM module
	HAL_Delay(6000);

	uint8_t counter = 0;
	do{
		HAL_Delay(2000);
		HAL_UART_Transmit(&huart1, command0, strlen(command0), 0xFFF); // Check for network
		counter++;
		if (counter>10)
			goto sending_failed;
	}while(!AT_expected_received((uint8_t*)"+CGREG: 0,1")); //means registered (home), details: http://www.groundcontrol.com/AT_Command_Reference_5_9_1_3.htm

	if(!AT_expected_received((uint8_t*)"OK"))
		goto sending_failed;

	HAL_UART_Transmit(&huart1, command1, strlen(command1), 0xFFF); // Select SMS Message Format
	if(!AT_expected_received((uint8_t*)"OK"))
		goto sending_failed;

	HAL_UART_Transmit(&huart1, command2, strlen(command2), 0xFFF); // Send Message
	//HAL_Delay(200);
	if(!AT_expected_received((uint8_t*)"> "))
		goto sending_failed;

	HAL_UART_Transmit(&huart1, text, length, 0xFFF);
	//HAL_Delay(200);

	uint8_t Ctrl_Z = 0x1A;
	HAL_UART_Transmit(&huart1, &Ctrl_Z, 1, 0xFFF);
	HAL_Delay(5000);

	//command used to print all unread messages to UART, for debug
	//uint8_t command_read_msgs[] = "AT+CMGL=\"REC UNREAD\"\r";
	//HAL_UART_Transmit(&huart1, command_read_msgs, strlen(command_read_msgs), 0xFFF);

	counter = 0;
	do{
		HAL_Delay(2000);
		if (counter++>10)
			goto sending_failed;
	}while(!AT_expected_received((uint8_t*)"OK"));

	HAL_GPIO_WritePin(GPIOA, GSM_POW_EN_Pin, GPIO_PIN_RESET);
	return 1;

#endif
sending_failed:
	HAL_GPIO_WritePin(GPIOA, GSM_POW_EN_Pin, GPIO_PIN_RESET);

	return 0;

	//uint8_t lol[] = "AT+sfdgsdfgsdfgsdfgsdghjgvjhgfxcbxcv\r";
	//HAL_UART_Transmit(&huart1, lol, strlen(lol), 0xFFF); //debug
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(GPIOA, DBG_LED_Pin|GSM_POW_EN_Pin, GPIO_PIN_SET);

  __enable_irq();

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  SET_BIT((&huart1)->Instance->CR3, USART_CR3_EIE);
  /* Enable the UART Data Register not empty Interrupt */
  SET_BIT((&huart1)->Instance->CR1, USART_CR1_RXNEIE);

  #ifdef SLEEP_ENABLED
  HAL_Delay(8000);
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flags_reg & 1<<ARMED_STATE){
		  if(flags_reg & 1<<USR_BTN_PRESSED){
			  flags_reg &= ~(1<<ARMED_STATE);
			  flags_reg &= ~(1<<PIR_SIGNAL_DETECTED);
			  flags_reg &= ~(1<<USB_VBUS_TOGGLED);
			  flags_reg &= ~(1<<USR_BTN_PRESSED);
			  for(uint8_t i = 0; i<4; i++){
				  HAL_GPIO_TogglePin(GPIOA, DBG_LED_Pin);
				  HAL_Delay(1000);
			  }
			  HAL_GPIO_WritePin(GPIOA, DBG_LED_Pin, GPIO_PIN_RESET);
		  }
		  if(flags_reg & 1<<PIR_SIGNAL_DETECTED){
			  uint8_t my_text[] = "MOTION DETECTED!!!";
			  send_SMS(my_text, strlen(my_text));
			  HAL_GPIO_WritePin(GPIOA, DBG_LED_Pin, GPIO_PIN_SET);
			  while(HAL_GPIO_ReadPin(PIR_SIGNAL_GPIO_Port, PIR_SIGNAL_Pin)){}
			  HAL_GPIO_WritePin(GPIOA, DBG_LED_Pin, GPIO_PIN_RESET);
			  flags_reg &= ~(1<<PIR_SIGNAL_DETECTED);
		  }
		  if(flags_reg & 1<<USB_VBUS_TOGGLED){
			  uint8_t text_on[] = "USB +5V turned on, charging";
			  uint8_t text_off[] = "USB +5V turned off, battery operation";
			  HAL_Delay(50);
			  if(HAL_GPIO_ReadPin(VBUS_DET_GPIO_Port, VBUS_DET_Pin)){
				  send_SMS(text_on, strlen(text_on));
			  }
			  else{
				  send_SMS(text_off, strlen(text_off));
			  }
			  flags_reg &= ~(1<<USB_VBUS_TOGGLED);
		  }
	  }
	  else{
		  if(flags_reg & 1<<USR_BTN_PRESSED){
			  flags_reg |= 1<<ARMED_STATE;
			  flags_reg &= ~(1<<PIR_SIGNAL_DETECTED);
			  flags_reg &= ~(1<<USB_VBUS_TOGGLED);
			  flags_reg &= ~(1<<USR_BTN_PRESSED);
			  for(uint8_t i = 0; i<32; i++){
				  HAL_GPIO_TogglePin(GPIOA, DBG_LED_Pin);
				  HAL_Delay(50);
			  }
			  HAL_GPIO_WritePin(GPIOA, DBG_LED_Pin, GPIO_PIN_RESET);
		  }
	  }

	  #ifdef SLEEP_ENABLED
	  HAL_SuspendTick();
	  //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	  #endif

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	#ifdef SLEEP_ENABLED
	HAL_ResumeTick();
	#endif

	switch(GPIO_Pin){
	case PIR_SIGNAL_Pin:
		flags_reg = flags_reg | 1<<PIR_SIGNAL_DETECTED;
		break;
	case USR_BTN_Pin:
		flags_reg = flags_reg | 1<<USR_BTN_PRESSED;
		break;
	case VBUS_DET_Pin:
		flags_reg = flags_reg | 1<<USB_VBUS_TOGGLED;
		break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
