/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**
 * @file  services_lib_interface.c
 * @brief Public interface for Services library
 * @note  Unique for each platform
 * @par
 */

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "services_lib_interface.h"
#include "services_lib_protocol.h"
#include "mhu.h"

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

#if DEVICE_REVISION == REV_B0
#define HE_DTCM_GLOBAL_ADDRESS      0x58800000ul
#else
#define HE_DTCM_GLOBAL_ADDRESS      0x60800000ul
#endif
#define HP_DTCM_GLOBAL_ADDRESS      0x50800000ul
#define M55_DTCM_LOCAL_OFFSET       0x20000000ul

//#define CPU M55_HP

//#define DTCM_GLOBAL_ADDRESS         HP_DTCM_GLOBAL_ADDRESS
/**
 * @note
 */
#if   CPU == M55_HE || defined(M55_HE)
#define DTCM_GLOBAL_ADDRESS         HE_DTCM_GLOBAL_ADDRESS
#elif CPU == M55_HP || defined(M55_HP)
#define DTCM_GLOBAL_ADDRESS         HP_DTCM_GLOBAL_ADDRESS
#else
#error Target CPU is not defined
#endif

#if PLATFORM_TYPE == SIMULATION_BOLT
/*
 * Never do semihosting when simulating
 */
#undef  SERVICES_PRINT_ENABLE
#define SERVICES_PRINT_ENABLE       1
#endif

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/

/*******************************************************************************
 *  G L O B A L   V A R I A B L E S
 ******************************************************************************/
static uint8_t
  s_packet_buffer[SERVICES_MAX_PACKET_BUFFER_SIZE] __attribute__ ((aligned (4)));

debug_print_function_t drv_debug_print_fn;

/*******************************************************************************
 *  C O D E
 ******************************************************************************/

/**
 * @brief Public interface for SERVICES delay function
 * @param wait_time_ms
 * @return
 * @note  User must supply this implementation for their platform. This is a
 *        bare metal use case
 */
int32_t SERVICES_wait_ms(uint32_t wait_time_ms)
{
  /*
   * To be filled in by the user
   */
  for (volatile int i = 0; i < wait_time_ms; i++)
  {
     /* Do nothing, but please do not optimse me out either */
     __asm__ volatile("nop");
  }

  return 0;
}

/**
 * @brief Public interface for SERVICES stub printing function
 * @param fmt
 * @note  Add you favourite printing method here
 */
int SERVICES_print(const char * fmt, ...)
{
#if SERVICES_PRINT_ENABLE != 0
  va_list args;
  char buffer[256];

  /*
   * @todo Handle long strings bigger than buffer size
   */
  va_start(args,fmt);
  vsprintf(buffer, fmt, args);
  va_end(args);

  printf("%s", buffer);
#else
  (void)fmt;
#endif // #if SERVICES_PRINT_ENABLE != 0

  return 0;
}

/**
 * @fn    SERVICES_Setup(MHU_send_message_t send_message)
 * @brief Public interface to initialize the SERVICES library
 *
 * @param send_message
 * @param timeout
 */
void SERVICES_Setup(MHU_send_message_t send_message, uint32_t timeout)
{
  services_lib_t  services_init_params =
  {
    .global_offset         = DTCM_GLOBAL_ADDRESS - M55_DTCM_LOCAL_OFFSET,
    .packet_buffer_address = (uint32_t)s_packet_buffer,
    .fn_send_mhu_message   = send_message,
    .fn_wait_ms            = &SERVICES_wait_ms,
    .wait_timeout          = timeout,
    .fn_print_msg          = &SERVICES_print,
  };
  drv_debug_print_fn = &SERVICES_print;

  SERVICES_initialize(&services_init_params);
}
