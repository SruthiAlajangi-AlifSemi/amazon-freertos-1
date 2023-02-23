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
 * @file     printf_to_uart.h
 * @author   Ganesh Ramani
 * @email    ganesh.ramani@alifsemi.com
 * @version  V1.0.0
 * @date
 * @brief    Redirect Printf statements to UART channel.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
#ifndef __PRINTF_TO_UART_H__
#define __PRINTF_TO_UART_H__

/* System Includes */
#include <stdio.h>

/* Enable Redirect printf to UART. */
#define PRINTF_REDIRECT_UART     0


/** APIs ******/

void UART_Printf_Init(void);

void UART_Printf_DeInit(void);


#endif //__PRINTF_TO_UART_H__
