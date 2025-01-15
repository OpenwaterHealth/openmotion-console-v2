/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device CDC ACM applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define APP_RX_DATA_SIZE   2048
#define APP_TX_DATA_SIZE   2048

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA      0x01
#define TX_NEW_TRANSMITTED_DATA   0x02

/* Data length for vcp */
#define VCP_WORDLENGTH8  8
#define VCP_WORDLENGTH9  9

/* the minimum baudrate */
#define MIN_BAUDRATE     115200

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM *cdc_acm;

#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = ".UsbxAppSection"
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION) /* ARM Compiler 5/6 */
__attribute__((section(".UsbxAppSection")))
#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".UsbxAppSection")))
#endif
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

UART_HandleTypeDef *uart_handler;
extern TX_EVENT_FLAGS_GROUP EventFlag;

UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
    cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;

    /* Set device class_cdc_acm with default parameters */
    if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                      &CDC_VCP_LineCoding) != UX_SUCCESS)
    {
      Error_Handler();
    }

  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  /* Reset the cdc acm instance */
  cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Function implementing usbx_cdc_acm_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input)
{
    ULONG actual_length;
    UX_SLAVE_DEVICE *device;
    device = &_ux_system_slave->ux_system_slave_device;

    while (1)
    {
        /* Check if device is configured */
        if((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
        {

            /* Read the received data in blocking mode */
            ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)UserRxBufferFS, 64, &actual_length);

            if (actual_length != 0)
            {
                /* Print received data as a string with specified length */
                printf("%.*s\r\n", (int)actual_length, (char *)UserRxBufferFS);
            }
        }
        else
        {
            /* Sleep thread for 10ms */
            tx_thread_sleep(MS_TO_TICK(10));
        }
    }
}

/**
  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input)
{
    /* Local Variables */
    ULONG actual_length;
    UX_SLAVE_DEVICE *device;
    ULONG myCount = 0;
    ULONG myLen = 0;
    device = &_ux_system_slave->ux_system_slave_device;
    /* Infinite Loop */
    while(1)
    {
              /* Check if device is configured */
              if((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
              {
            	  	  memset(UserTxBufferFS, 0, APP_TX_DATA_SIZE);

                      /* Format the data to send */
                      snprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, "Count: %lu\r\n", myCount);

                      /* Calculate the length of the string to send */
                      myLen = strlen((char *)UserTxBufferFS);

                      /* Send the data over CDC ACM */
                      ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)UserTxBufferFS, myLen, &actual_length);

                      /* Increment the counter */
                      myCount++;

                      /* Sleep for 1s */
                      tx_thread_sleep(100);
              }
              else
              {
                  /* Sleep briefly if the device is not configured */
                  tx_thread_sleep(10);
              }
    }
}

/* USER CODE END 1 */
