/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     printf_to_uart.c
 * @author   Ganesh Ramani
 * @email    ganesh.ramani@alifsemi.com
 * @version  V1.0.0
 * @date
 * @brief    Redirect Printf statements to UART channel.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* include for UART Driver */
#include "Driver_USART.h"
#include "UART_dev.h"

/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"

#include "printf_to_uart.h"

/* UART Driver instance (UART0-UART7) */
#define UART      4

/*Define for FreeRTOS objects */
#define UART_RX_CB_EVENT                       0x01
#define UART_TX_CB_EVENT                       0x02


/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);


#if PRINTF_REDIRECT_UART

void uartWrite(char c); /* Used by fputc() */
char uartRead(void);    /* Used by fgetc() */

FILE __stdout;
FILE __stdin;

int fputc(int c, FILE * stream)
{
    uartWrite(c);
    /* return the character written to denote a successful write */
    return c;
}

int fgetc(FILE * stream)
{
    char c = uartRead();
    /* To echo Received characters back to serial Terminal */
    uartWrite(c);
    return c;
}

void uartWrite(char c) //Used by fputc()
{
    uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)UART4_BASE;

    /* wait until uart is to ready to send */
    while ( (uart_reg_ptr->usr & UART_USR_TRANSMIT_FIFO_NOT_FULL) == 0 );

    /* write a char to thr transmit holding register */
    uart_reg_ptr->rbr_thr_dll = c;
}

char uartRead(void) //Used by fgetc()
{
    /* Device specific code to Receive a byte from RX pin.
     * return received chararacter(byte)
     */

    uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)UART4_BASE;

    /* wait until uart is ready to receive */
    while ( (uart_reg_ptr->usr & UART_USR_RECEIVE_FIFO_NOT_EMPTY) == 0 );

    /* read a char from receive buffer register */
    return (int32_t)uart_reg_ptr->rbr_thr_dll;
}

#endif /* END of PRINTF_REDIRECT_UART */

/**
 * @function    INT hardware_init(void)
 * @brief   UART hardware pin initialization using PIN-MUX driver
 * @note    none
 * @param   void
 * @retval  execution status
 */
static int hardware_init(void)
{
    int ret = ARM_DRIVER_OK;

    /* PINMUX UART4_RX_B */
    ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_1);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART PINMUX. */
        return ret;
    }

    /* PINMUX UART4_TX_B */
    ret = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_2, PINMUX_ALTERNATE_FUNCTION_1);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART PINMUX. */
        return ret;
    }

    return ret;
}

/**
 * @function    void UART_callback(UINT event)
 * @brief       UART ISR callabck
 * @note        none
 * @param       event: USART Event
 * @retval      none
 */
static void UART_callback_(uint32_t event)
{
   return;
}

/**
 * @function    UART_Printf_Init
 * @brief       Initialize UART interface.
 * @note        none
 * @param       none
 * @retval      none
 */
void UART_Printf_Init(void)
{
    char  cmd    = 0;
    int   num    = 0;
    int   ret    = 0;
    ARM_DRIVER_VERSION version;

    version = USARTdrv->GetVersion();

    /* Initialize UART hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART hardware_init. */
        return;
    }

    /* Initialize UART driver */
    ret = USARTdrv->Initialize(UART_callback_);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Initialize. */
        return;
    }

    /* Power up UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Power Up. */
        goto error_uninitialize;
    }

    /* Configure UART to 115200 Bits/sec */
    ret = USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                          ARM_USART_DATA_BITS_8       |
                          ARM_USART_PARITY_NONE       |
                          ARM_USART_STOP_BITS_1       |
                          ARM_USART_FLOW_CONTROL_NONE, 115200);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Control. */
        goto error_poweroff;
    }

    /* Enable Receiver and Transmitter lines */
    ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Control TX. */
        goto error_poweroff;
    }

    ret =  USARTdrv->Control(ARM_USART_CONTROL_RX, 1);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Control RX. */
        goto error_poweroff;
    }

    //No Error
    return;

error_poweroff:

    /* Received error Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Power OFF. */
        return;
    }

error_uninitialize:

    /* Received error Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Uninitialize. */
        return;
    }
}


/**
 * @function    UART_Printf_DeInit
 * @brief       De-Initialize UART interface.
 * @note        none
 * @param       none
 * @retval      none
 */
void UART_Printf_DeInit(void)
{
    int ret = 0;

    /* Received error Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Power OFF. */
        return;
    }

    /* Received error Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        /* Error in UART Uninitialize. */
        return;
    }

    return;
}
