/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/* Copyright (c) 2022 ALIF SEMICONDUCTOR

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

/******************************************************************************
 * @file     entropy_platform_poll.c
 * @author   Sruthi Alajangi
 * @email    sruthi.alajangi@alifsemi.com
 * @version  V1.0.0
 * @date     21-Sep-2022
 * @brief    This file provides code for the entropy collector.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* mbed TLS includes. */
#include "mbedtls/config.h"
#include "threading_alt.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ssl.h"
#include "mbedtls/error.h"

#include "SE_Service_Int_MHU.h"
/*-----------------------------------------------------------*/
/**
 * @brief The number of bytes in a standard TLS record header.
 *
 * @note This only applies to a standard TCP+TLS connection.
 *       DTLS has a different length for its record headers.
 *
 */
#define TLS_RECORD_HEADER_BYTE_LENGTH    5

static UBaseType_t ulNextRand1;

UBaseType_t uxRand1( void )
{
    const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;
    ulNextRand1 = ( ulMultiplier * ulNextRand1 ) + ulIncrement;
    return( ( int ) ( ulNextRand1 ) & 0x7fffUL );
}

/*-----------------------------------------------------------*/

/**
 * @brief Allocates memory for an array of members.
 *
 * @param[in] nmemb Number of members that need to be allocated.
 * @param[in] size Size of each member.
 *
 * @return Pointer to the beginning of newly allocated memory.
 */
void * mbedtls_platform_calloc( size_t nmemb,
                                size_t size )
{
    size_t totalSize = nmemb * size;
    void * pBuffer = NULL;
    /* Check that neither nmemb nor size were 0. */
    if( totalSize > 0 )
    {
    /* Overflow check. */
       if( ( totalSize / size ) == nmemb )
       {
           pBuffer = pvPortMalloc( totalSize );
           if( pBuffer != NULL )
           {
               ( void ) memset( pBuffer, 0x00, totalSize );
           }
       }
    }
    return pBuffer;
}

/*-----------------------------------------------------------*/

/**
 * @brief Frees the space previously allocated by calloc.
 *
 * @param[in] ptr Pointer to the memory to be freed.
 */
void mbedtls_platform_free( void * ptr )
{
    vPortFree( ptr );
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/**
 * @brief Frees a mutex.
 *
 * @param[in] pMutex mbedtls mutex handle.
 *
 * @note This function is an empty stub as nothing needs to be done to free
 * a statically allocated FreeRTOS mutex.
 */
void mbedtls_platform_mutex_free( mbedtls_threading_mutex_t * pMutex )
{
    /* Nothing needs to be done to free a statically-allocated FreeRTOS mutex. */
    ( void ) pMutex;
}

/*-----------------------------------------------------------*/
unsigned char EntropyBuffer[128];
/**
 * @brief Function to generate a random number.
 *
 * @param[in] data Callback context.
 * @param[out] output The address of the buffer that receives the random number.
 * @param[in] len Maximum size of the random number to be generated.
 * @param[out] olen The size, in bytes, of the #output buffer.
 *
 * @return 0 if no critical failures occurred,
 * MBEDTLS_ERR_ENTROPY_SOURCE_FAILED otherwise.
 */
int mbedtls_platform_entropy_poll( void *data,unsigned char *output, size_t len, size_t *olen )
{
	uint32_t rndValue = 0;
	int32_t  eCode = 0;

    SE_Service_MHU_GetRandom_Value(sizeof(rndValue), &rndValue, &eCode);
    *olen  = sizeof(rndValue);
	SERVICES_print("SE Generated Random Number %d \n", rndValue );

    return eCode;
}

/*-----------------------------------------------------------*/

/**
 * @brief Function to generate a random number based on a hardware poll.
 *
 * For this FreeRTOS Windows port, this function is redirected by calling
 * #mbedtls_platform_entropy_poll.
 *
 * @param[in] data Callback context.
 * @param[out] output The address of the buffer that receives the random number.
 * @param[in] len Maximum size of the random number to be generated.
 * @param[out] olen The size, in bytes, of the #output buffer.
 *
 * @return 0 if no critical failures occurred,
 * MBEDTLS_ERR_ENTROPY_SOURCE_FAILED otherwise.
 */
int mbedtls_hardware_poll( void * data,unsigned char * output,
                           size_t len,size_t * olen )
{
    return mbedtls_platform_entropy_poll( data, output, len, olen );
}

