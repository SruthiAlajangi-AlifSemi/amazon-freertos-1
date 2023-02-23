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
 * @file     SE_Service_Int_MHU.c
 * @author   Ganesh Ramani
 * @email    ganesh.ramani@alifsemi.com
 * @version  V1.0.0
 * @date
 * @brief    SE Service Wrapper
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include <stdio.h>
#include <inttypes.h>
#include <stdarg.h>
#include <time.h>

#include "mhu.h"
#include "mhu_driver.h"
#include "SE_Service_Int_MHU.h"
#include "services_lib_interface.h"

#include "M55_HP.h"

//#define CPU = M55_HP
/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

#if   CPU == M55_HE
#define CPU_STRING "M55_HE"
#elif CPU == M55_HP
#define CPU_STRING "M55_HP"
#else
#define CPU_STRING "<unknown>"
#endif

//#define CPU_STRING "M55_HP"

/**
 * Test parameters
 * CONTINUOUS_RUN Will run forever if set to 1
 * LIMITED_RUN    Default is 1 so just once around
 */
#define CONTINUOUS_RUN         0
#define LIMITED_RUN            1

#define SE_MHU0_RECV_BASE      0x40040000
#define SE_MHU0_SEND_BASE      0x40050000

#define SE_MHU1_RECV_BASE      0x40060000
#define SE_MHU1_SEND_BASE      0x40070000

#define NUM_MHU                2

#define MHU_M55_SE_MHU0        0
#define MHU_M55_SE_MHU1        1

#define SEES_MHU0_COMB_IRQn    37
#define ESSE_MHU0_COMB_IRQn    38
#define SEES_MHU1_COMB_IRQn    39
#define ESSE_MHU1_COMB_IRQn    40

#define MAXIMUM_TIMEOUT        0x01000000

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/


/*******************************************************************************
 *  G L O B A L   V A R I A B L E S
 ******************************************************************************/

uint32_t sender_base_address_list[NUM_MHU] =
{
  SE_MHU0_SEND_BASE,
  SE_MHU1_SEND_BASE,
};

uint32_t receiver_base_address_list[NUM_MHU] =
{
  SE_MHU0_RECV_BASE,
  SE_MHU1_RECV_BASE,
};

static mhu_driver_in_t s_mhu_driver_in;
static mhu_driver_out_t s_mhu_driver_out;

static uint32_t SE_SERV_handle = 0;

/*******************************************************************************
 *  C O D E
 ******************************************************************************/

/**
 * @brief MHU0 TX IRQ handler
 */
//void MHU_ES0SE0_TX_IRQHandler(void)
//{
//  s_mhu_driver_out.sender_irq_handler(MHU_M55_SE_MHU0);
//}
void MHU_SESS_S_TX_IRQHandler(void)
{
  s_mhu_driver_out.sender_irq_handler(MHU_M55_SE_MHU0);
}

/**
 * @brief MHU0 RX IRQ handler
 */
//void MHU_SEES00_RX_IRQHandler(void)
//{
//  s_mhu_driver_out.receiver_irq_handler(MHU_M55_SE_MHU0);
//}
void MHU_SESS_S_RX_IRQHandler(void)
{
  s_mhu_driver_out.receiver_irq_handler(MHU_M55_SE_MHU0);
}


/**
 * @brief MHU1 TX IRQ handler
 */
//void MHU_ES0SE1_TX_IRQHandler(void)
//{ printf("----MHU_ES0SE1_TX_IRQHandler----\n");
//  s_mhu_driver_out.sender_irq_handler(MHU_M55_SE_MHU1);
//}
void MHU_SESS_NS_TX_IRQHandler(void)
{ printf("----MHU_SESS_NS_TX_IRQHandler----\n");
  s_mhu_driver_out.sender_irq_handler(MHU_M55_SE_MHU1);
}
/**
 * @brief MHU1 RX IRQ handler
 */
//void MHU_SEES01_RX_IRQHandler(void)
//{ printf("---MHU_SEES01_RX_IRQHandler----\n");
//  s_mhu_driver_out.receiver_irq_handler(MHU_M55_SE_MHU1);
//}
void MHU_SESS_NS_RX_IRQHandler(void)
{ printf("---MHU_SESS_NS_RX_IRQHandler----\n");
  s_mhu_driver_out.receiver_irq_handler(MHU_M55_SE_MHU1);
}

/**
 * @brief Function to set up IRQs
 * @param irq_num
 * @param irq_handler
 */
static void setup_irq(int irq_num, uint32_t irq_handler)
{
  NVIC_DisableIRQ(irq_num);
  NVIC_ClearPendingIRQ(irq_num);
  // Place the interrupt handler directly in the vector table (for XIP case)
//NVIC_SetVector(irq_num, irq_handler);
  NVIC_SetPriority(irq_num, 2);
  NVIC_EnableIRQ(irq_num);
}

/**
 * @brief MHU initialize function
 */
//static void mhu_initialize(void)
//{
//  s_mhu_driver_in.sender_base_address_list = sender_base_address_list;
//  s_mhu_driver_in.receiver_base_address_list = receiver_base_address_list;
//  s_mhu_driver_in.mhu_count = NUM_MHU;
//  s_mhu_driver_in.send_msg_acked_callback = SERVICES_send_msg_acked_callback;
//  s_mhu_driver_in.rx_msg_callback = SERVICES_rx_msg_callback;
//  s_mhu_driver_in.debug_print = SERVICES_print;
//
//  MHU_driver_initialize(&s_mhu_driver_in, &s_mhu_driver_out);
//
//  setup_irq(SEES_MHU0_COMB_IRQn, (uint32_t)&MHU_SEES00_RX_IRQHandler);
//  setup_irq(ESSE_MHU0_COMB_IRQn, (uint32_t)&MHU_ES0SE0_TX_IRQHandler);
//
//  setup_irq(SEES_MHU1_COMB_IRQn, (uint32_t)&MHU_SEES01_RX_IRQHandler);
//  setup_irq(ESSE_MHU1_COMB_IRQn, (uint32_t)&MHU_ES0SE1_TX_IRQHandler);
//}
static void mhu_initialize(void)
{
  s_mhu_driver_in.sender_base_address_list = sender_base_address_list;
  s_mhu_driver_in.receiver_base_address_list = receiver_base_address_list;
  s_mhu_driver_in.mhu_count = NUM_MHU;
  s_mhu_driver_in.send_msg_acked_callback = SERVICES_send_msg_acked_callback;
  s_mhu_driver_in.rx_msg_callback = SERVICES_rx_msg_callback;
  s_mhu_driver_in.debug_print = SERVICES_print;

  MHU_driver_initialize(&s_mhu_driver_in, &s_mhu_driver_out);

  setup_irq(SEES_MHU0_COMB_IRQn, (uint32_t)&MHU_SESS_S_RX_IRQHandler);
  setup_irq(ESSE_MHU0_COMB_IRQn, (uint32_t)&MHU_SESS_S_TX_IRQHandler);

  setup_irq(SEES_MHU1_COMB_IRQn, (uint32_t)&MHU_SESS_NS_RX_IRQHandler);
  setup_irq(ESSE_MHU1_COMB_IRQn, (uint32_t)&MHU_SESS_NS_TX_IRQHandler);
}


/**
 * @ MHU Initialize
 */
void SE_Service_MHU_Init(void)
{
    mhu_initialize();

    SERVICES_Setup(s_mhu_driver_out.send_message, MAXIMUM_TIMEOUT);

    //SERVICES_wait_ms(0x1000000);

    //Get Handle.
    SE_SERV_handle = SERVICES_register_channel(MHU_M55_SE_MHU0, 0);

    return;
}


void SE_Service_MHU_GetRandom_Value(uint32_t length, void* rndValue, int32_t *eCode) {

    SERVICES_cryptocell_get_rnd(SE_SERV_handle, length, rndValue, eCode);

    return;
}


void SE_Service_MHU_Reset_SoC(void) {

    SERVICES_boot_reset_soc(SE_SERV_handle);

    return;
}


/***
 * @ MHU re-initialize
 */
void SE_Service_MHU_DeInit(void)
{
    return;
}
