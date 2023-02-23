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
 * @file     SE_Service_Int_MHU.h
 * @author   Ganesh Ramani
 * @email    ganesh.ramani@alifsemi.com
 * @version  V1.0.0
 * @date
 * @brief    Wrapper for SE Service APIs
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef SE_HOST_SERVICE_INCLUDE_SE_SERVICE_INT_MHU_H_
#define SE_HOST_SERVICE_INCLUDE_SE_SERVICE_INT_MHU_H_

void SE_Service_MHU_Init(void);

void SE_Service_MHU_GetRandom_Value(uint32_t length, void* rndValue, int32_t *eCode) ;

void SE_Service_MHU_Reset_SoC(void);

void SE_Service_MHU_DeInit(void);

#endif /* SE_HOST_SERVICE_INCLUDE_SE_SERVICE_INT_MHU_H_ */
