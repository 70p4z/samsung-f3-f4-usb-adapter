/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

#define HOST_USB
#define USART_INTF USART2
#define TXEN_Pin GPIO_PIN_1
#define TXEN_Port GPIOA

#define LEDTX_Pin GPIO_PIN_6
#define LEDTX_Port GPIOB

#define LEDRX_Pin GPIO_PIN_7
#define LEDRX_Port GPIOB

#define LED_TIMEOUT 100

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART_UART_Init(void);
/* USER CODE BEGIN PFP */

#define USB_RX_LATENCY_MS 20

uint32_t upgrade_mode;
uint32_t usb_configured;
uint8_t usb_rx[CDC_DATA_FS_MAX_PACKET_SIZE];
uint32_t usb_rx_len;
uint32_t usb_rx_fwd_off;
uint8_t usb_tx[CDC_DATA_FS_MAX_PACKET_SIZE];
uint8_t usb_tx_bg[CDC_DATA_FS_MAX_PACKET_SIZE];
uint32_t usb_tx_off;
uint32_t usb_tx_lastadd;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  if (usb_rx == Buf) {
    usb_rx_len = *Len;
    usb_rx_fwd_off=0;
  }
  return (0);
}

void main_cdc_init(void) {
  usb_configured = 1;
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_rx);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART_UART_Init();
  usb_configured = 0;
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // nothing to transmit so far
  usb_tx_off=0;
  usb_tx_lastadd=0;
  // nothing received so far
  usb_rx_len=usb_rx_fwd_off=0;

  HAL_GPIO_WritePin(LEDTX_Port, LEDTX_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEDRX_Port, LEDRX_Pin, GPIO_PIN_SET);

  uint32_t ledtx_timeout = uwTick + LED_TIMEOUT;
  uint32_t ledrx_timeout = uwTick + LED_TIMEOUT;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    if (LL_USART_IsActiveFlag_RXNE(USART_INTF)) {
      // read and append character to the usb buffer
      uint8_t b = LL_USART_ReceiveData8(USART_INTF);
      // and send it
      usb_tx[usb_tx_off++] = b;

      HAL_GPIO_WritePin(LEDRX_Port, LEDRX_Pin, GPIO_PIN_SET);
      ledrx_timeout = uwTick + LED_TIMEOUT;
      if (!ledrx_timeout) ledrx_timeout++;
      
      if (usb_tx_off==sizeof(usb_tx)
        && usb_configured) {
        // immediate sending
        memmove(usb_tx_bg, usb_tx, usb_tx_off);
        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_tx_bg, usb_tx_off);
        USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        usb_tx_off=0;
        usb_tx_lastadd=0;
      }
      else {
        // send after timeout (not to make the usb too busy)
        usb_tx_lastadd=uwTick+USB_RX_LATENCY_MS;
        if (usb_tx_lastadd == 0) usb_tx_lastadd++;
      }
    }

    // expired time to fwd the received data?
    if (usb_tx_off && usb_tx_lastadd && uwTick - usb_tx_lastadd < 0x80000000U
      && usb_configured) {
      usb_tx_lastadd = 0;
      memmove(usb_tx_bg, usb_tx, usb_tx_off);
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_tx_bg, usb_tx_off);
      USBD_CDC_TransmitPacket(&hUsbDeviceFS);
      usb_tx_off=0;
    }

    // transmit data from usb_rx toward usart
    if (LL_USART_IsActiveFlag_TXE(USART_INTF)) {
      if (usb_rx_len 
        && usb_rx_fwd_off < usb_rx_len) {
        
        HAL_GPIO_WritePin(LEDTX_Port, LEDTX_Pin, GPIO_PIN_SET);
        ledtx_timeout = uwTick + LED_TIMEOUT;
        if (!ledtx_timeout) ledtx_timeout++;

        if (upgrade_mode) {
          WRITE_REG(FLASH->KEYR, FLASH_KEY1);
          WRITE_REG(FLASH->KEYR, FLASH_KEY2);
          /* Only bank1 will be erased*/
          SET_BIT(FLASH->CR, FLASH_CR_MER);
          SET_BIT(FLASH->CR, FLASH_CR_STRT);
          FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
          NVIC_SystemReset();
        }

        uint32_t timeout = uwTick + 1000;
        // transmission will occur on RS485
        HAL_GPIO_WritePin(TXEN_Port, TXEN_Pin, GPIO_PIN_SET);
        // send byte
        LL_USART_TransmitData8(USART_INTF, usb_rx[usb_rx_fwd_off++]);
        // wait transmission ended
        while (!LL_USART_IsActiveFlag_TC(USART_INTF)) { 
          if (uwTick - timeout < 0x80000000U) {
            break;
          }
        }
        // transmission ended, back to RX on RS485
        HAL_GPIO_WritePin(TXEN_Port, TXEN_Pin, GPIO_PIN_RESET);
        // prepare reception of next usb chunk
        if (usb_rx_fwd_off==usb_rx_len) {
          USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        }
      }
    }

    if (ledrx_timeout &&  uwTick - ledrx_timeout < 0x80000000UL) {
      ledrx_timeout = 0;
      HAL_GPIO_WritePin(LEDRX_Port, LEDRX_Pin, GPIO_PIN_RESET);
    }

    if (ledtx_timeout &&  uwTick - ledtx_timeout < 0x80000000UL) {
      ledtx_timeout = 0;
      HAL_GPIO_WritePin(LEDTX_Port, LEDTX_Pin, GPIO_PIN_RESET);
    }

    if (LL_USART_IsActiveFlag_ORE(USART_INTF)) {
      LL_USART_ClearFlag_ORE(USART_INTF);
    }
    if (LL_USART_IsActiveFlag_NE(USART_INTF)) {
      LL_USART_ClearFlag_NE(USART_INTF);
    }
    if (LL_USART_IsActiveFlag_FE(USART_INTF)) {
      LL_USART_ClearFlag_FE(USART_INTF);
    }
    if (LL_USART_IsActiveFlag_PE(USART_INTF)) {
      LL_USART_ClearFlag_PE(USART_INTF);
    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart.Instance = USART_INTF;
  huart.Init.BaudRate = 9600;
  huart.Init.WordLength = UART_WORDLENGTH_9B; /*include parity bit*/
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_EVEN;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

  LL_USART_Enable(USART_INTF);

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
  //CONCAT(__HAL_RCC_, CONCAT(TXEN_Port, _CLK_ENABLE)) ();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TXEN_Port, TXEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEDTX_Port, LEDTX_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEDRX_Port, LEDRX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = TXEN_Pin;
  GPIO_InitStruct.Mode = TXEN_PIN_MODE;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TXEN_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEDTX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDTX_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEDRX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDRX_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  NVIC_SystemReset();
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
