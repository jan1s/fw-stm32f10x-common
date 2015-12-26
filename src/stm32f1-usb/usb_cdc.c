/**
  ******************************************************************************
  * @file    hw_config.c
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


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_cdc.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE];
uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;

uint8_t  USB_Rx_Buffer [USB_RX_DATA_SIZE];
uint32_t USB_Rx_ptr_in = 0;
uint32_t USB_Rx_ptr_out = 0;
uint32_t USB_Rx_length  = 0;

uint8_t  USB_Tx_State = 0;
/* Extern variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : USB_CDC_Init
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_CDC_Init(void)
{
    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClearITPendingBit(EXTI_Line18);
    EXTI_InitStructure.EXTI_Line = EXTI_Line18;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Select USBCLK source */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

    /* Enable the USB clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

    /* Configure USB interrupts */
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 2 bit for pre-emption priority, 2 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    /* Init USB Stack */
    USB_Init();
}

/*******************************************************************************
* Function Name  : USB_CDC_FillRxBuffer.
* Description    : send the received data from USB to the internal buffer.
* Input          : data_buffer: data address.
                   Nb_bytes: number of bytes to send.
* Return         : none.
*******************************************************************************/
void USB_CDC_FillRxBuffer(uint8_t* data_buffer, uint8_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        if(USB_Rx_ptr_in >= USB_RX_DATA_SIZE)
        {
            USB_Rx_ptr_in = 0;
        }
        USB_Rx_Buffer[USB_Rx_ptr_in] = data_buffer[i];
        USB_Rx_ptr_in++;
        USB_Rx_length++;
    }
}

/*******************************************************************************
* Function Name  : USB_CDC_Read.
* Description    : get the received data from the internal buffer.
* Input          : c: call by reference get first byte from buffer.
* Return         : bytes left in buffer, or -1 if empty.
*******************************************************************************/
uint32_t USB_CDC_Read(uint8_t* c)
{
    uint32_t available = USB_Rx_length;
    if(USB_Rx_length > 0)
    {
        (*c) = USB_Rx_Buffer[USB_Rx_ptr_out];
        USB_Rx_ptr_out++;
        if(USB_Rx_ptr_out >= USB_RX_DATA_SIZE)
        {
            USB_Rx_ptr_out = 0;
        }
        USB_Rx_length--;
    }
    return available;
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USB_CDC_Handle_USBAsynchXfer (void)
{
    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if(USB_Tx_State != 1)
    {
        if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
        {
            USART_Rx_ptr_out = 0;
        }

        if(USART_Rx_ptr_out == USART_Rx_ptr_in)
        {
            USB_Tx_State = 0;
            return;
        }

        if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
        {
            USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
        }
        else
        {
            USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
        }

        if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
        {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

            USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
            USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
        }
        else
        {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = USART_Rx_length;

            USART_Rx_ptr_out += USART_Rx_length;
            USART_Rx_length = 0;
        }
        USB_Tx_State = 1;
        UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
        SetEPTxCount(ENDP1, USB_Tx_length);
        SetEPTxValid(ENDP1);
    }

}
/*******************************************************************************
* Function Name  : USB_CDC_Send.
* Description    : send the received data from local buffer to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USB_CDC_Send(uint8_t c)
{
    USART_Rx_Buffer[USART_Rx_ptr_in] = c;
    USART_Rx_ptr_in++;

    /* To avoid buffer overflow */
    if(USART_Rx_ptr_in == USART_RX_DATA_SIZE)
    {
        USART_Rx_ptr_in = 0;
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
