/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CDC_H
#define __USB_CDC_H

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040
#define USART_RX_DATA_SIZE    2048
#define USB_RX_DATA_SIZE      2048

/* Exported functions ------------------------------------------------------- */
void USB_CDC_Init(void);
uint32_t USB_CDC_Read(uint8_t* c);
void USB_CDC_Send(uint8_t c);
void USB_CDC_FillRxBuffer(uint8_t* data_buffer, uint8_t length);
void USB_CDC_Handle_USBAsynchXfer (void);

/* External variables --------------------------------------------------------*/

#endif  /*__USB_CDC_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
