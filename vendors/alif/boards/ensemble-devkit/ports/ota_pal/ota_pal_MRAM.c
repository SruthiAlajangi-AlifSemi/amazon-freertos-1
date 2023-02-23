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
 * @file     ota_pal_MRAM.c
 * @author   Ganesh Ramani
 * @email    ganesh.ramani@alifsemi.com
 * @version  V1.0.0
 * @date
 * @brief    Platform Abstraction layer for AWS OTA Service Demonstration,
 *           Downloading file will be kept in MRAM.
 * @bug      None.
 * @Note     Restart, Cod Sign Validation not done.
 ******************************************************************************/

/* System Includes */
//#include <RTE_Components.h>
//#include CMSIS_device_header
#include "M55_HP.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/* OTA PAL Port include. */
#include "ota_pal.h"

/* MRAM Flash Driver */
#include "Driver_Flash.h"

#include "SE_Service_Int_MHU.h"

/* Valid MRAM Flash address. */
#define MRAM_ADDR           (MRAM_BASE + 0x100)

/* MRAM Flash Driver instance(default 0th instance for MRAM) */
#define FLASH_MRAM_DRV_INSTANCE          0

/* MRAM Flash Driver */
extern ARM_DRIVER_FLASH ARM_Driver_Flash_(FLASH_MRAM_DRV_INSTANCE);

/* MRAM Flash Driver instance */
static ARM_DRIVER_FLASH *FLASH_MRAM_drv = &ARM_Driver_Flash_(FLASH_MRAM_DRV_INSTANCE);

/* Mock value to know device restarted */
static int RESTARTED = 0;

/***********************************************************************
 *
 * Variables
 *
 **********************************************************************/
 /**
 * @brief File Signature Key
 *
 * The OTA signature algorithm we support on this platform.
 */
const char OTA_JsonFileSignatureKey[ OTA_FILE_SIG_KEY_STR_MAX_LENGTH ] = "sig-sha256-rsa";


/**
 * @brief Ptr to system context
 *
 * Keep track of system context between calls from the OTA Agent
 *
 */
const OtaFileContext_t * pxSystemContext = NULL;

/***********************************************************************
 *
 * Functions
 *
 **********************************************************************/

/**
 * @brief Abort an OTA transfer.
 *
 * Aborts access to an existing open file represented by the OTA file context pFileContext. This is
 * only valid for jobs that started successfully.
 *
 * @note The input OtaFileContext_t pFileContext is checked for NULL by the OTA agent before this
 * function is called.
 *
 * This function may be called before the file is opened, so the file pointer pFileContext->fileHandle
 * may be NULL when this function is called.
 *
 * @param[in] pFileContext OTA file context information.
 *
 * @return The OtaPalStatus_t error code is a combination of the main OTA PAL interface error and
 *         the MCU specific sub error code. See ota_platform_interface.h for the OtaPalMainStatus_t
 *         error codes and your specific PAL implementation for the sub error code.
 *
 * Major error codes returned are:
 *
 *   OtaPalSuccess: Aborting access to the open file was successful.
 *   OtaPalFileAbort: Aborting access to the open file context was unsuccessful.
 */
OtaPalStatus_t otaPal_Abort( OtaFileContext_t * const pFileContext )
{
	printf( "OTA PAL Abort invoked...\n" );

    if( (pFileContext == NULL) || ((pFileContext != pxSystemContext ) && ( pxSystemContext != NULL ) ) )
    {
        return OTA_PAL_COMBINE_ERR( OtaPalAbortFailed, 0 );
    }

    if( pxSystemContext == NULL )
    {
        return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
    }

    return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}

/**
 * @brief Create a new receive file.
 *
 * @note Opens the file indicated in the OTA file context in the MCU file system.
 *
 * @note The previous image may be present in the designated image download partition or file, so the
 * partition or file must be completely erased or overwritten in this routine.
 *
 * @note The input OtaFileContext_t pFileContext is checked for NULL by the OTA agent before this
 * function is called.
 * The device file path is a required field in the OTA job document, so pFileContext->pFilePath is
 * checked for NULL by the OTA agent before this function is called.
 *
 * @param[in] pFileContext OTA file context information.
 *
 * @return The OtaPalStatus_t error code is a combination of the main OTA PAL interface error and
 *         the MCU specific sub error code. See ota_platform_interface.h for the OtaPalMainStatus_t
 *         error codes and your specific PAL implementation for the sub error code.
 *
 * Major error codes returned are:
 *
 *   OtaPalSuccess: File creation was successful.
 *   OtaPalRxFileTooLarge: The OTA receive file is too big for the platform to support.
 *   OtaPalBootInfoCreateFailed: The bootloader information file creation failed.
 *   OtaPalRxFileCreateFailed: Returned for other errors creating the file in the device's
 *                             non-volatile memory. If this error is returned, then the sub error
 *                             should be set to the appropriate platform specific value.
 */
OtaPalStatus_t otaPal_CreateFileForRx( OtaFileContext_t * const pFileContext )
{

	int ret  = 0;

    if( pFileContext == NULL || pFileContext->pFilePath == NULL )
    {
        return OTA_PAL_COMBINE_ERR( OtaPalRxFileCreateFailed, 0 );
    }

    printf( "MRAM driver initialization \n" );

	/* Initialize MRAM Flash driver */
	ret = FLASH_MRAM_drv->Initialize(NULL);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Initialize.\r\n");
		return OTA_PAL_COMBINE_ERR( OtaPalUninitialized, 0 );;
	}

	/* Power up Flash peripheral */
	ret = FLASH_MRAM_drv->PowerControl(ARM_POWER_FULL);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Power Up.\r\n");
		return OTA_PAL_COMBINE_ERR( OtaPalUninitialized, 0 );
	}

	//Initialization Success.
    pxSystemContext = pFileContext;

    pFileContext->pFile = (uint8_t*) 0x01;

    printf(" \r\n Image size %d\r\n", pFileContext->fileSize);

    return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}

static OtaPalStatus_t otaPal_CheckSignature( OtaFileContext_t * const pFileContext )
{

    //TODO: Signature Check
#if 0
	unsigned char output1[32] = {0};
	int ret = 0;

	//Get Checksum
	ret = mbedtls_sha256_ret(MRAM_ADDR, pFileContext->fileSize, output1, 0);
	printf("Return value from mbedtls_sha256_ret [%d] \n", ret);

	//Encrypt with public key and check the output.

	mbedtls_pk_decrypt(ctx, input, ilen, output, olen, osize, f_rng, p_rng);

#endif

	return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}

/**
 * @brief Authenticate and close the underlying receive file in the specified OTA context.
 *
 * @note The input OtaFileContext_t pFileContext is checked for NULL by the OTA agent before this
 * function is called. This function is called only at the end of block ingestion.
 * otaPAL_CreateFileForRx() must succeed before this function is reached, so
 * pFileContext->fileHandle(or pFileContext->pFile) is never NULL.
 * The file signature key is required job document field in the OTA Agent, so pFileContext->pSignature will
 * never be NULL.
 *
 * If the signature verification fails, file close should still be attempted.
 *
 * @param[in] pFileContext OTA file context information.
 *
 * @return The OtaPalStatus_t error code is a combination of the main OTA PAL interface error and
 *         the MCU specific sub error code. See ota_platform_interface.h for the OtaPalMainStatus_t
 *         error codes and your specific PAL implementation for the sub error code.
 *
 * Major error codes returned are:
 *
 *   OtaPalSuccess on success.
 *   OtaPalSignatureCheckFailed: The signature check failed for the specified file.
 *   OtaPalBadSignerCert: The signer certificate was not readable or zero length.
 *   OtaPalFileClose: Error in low level file close.
 */
OtaPalStatus_t otaPal_CloseFile( OtaFileContext_t * const pFileContext )
{
	int32_t ret = 0;

    //check the flashed image Sign.
    otaPal_CheckSignature( pFileContext );

	ret = FLASH_MRAM_drv->PowerControl(ARM_POWER_OFF);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Power OFF.\r\n");
	}

	ret = FLASH_MRAM_drv->Uninitialize();
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error in MRAM Flash Uninitialized.\r\n");
	}

    return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}


/**
 * @brief Write a block of data to the specified file at the given offset.
 *
 * @note The input OtaFileContext_t pFileContext is checked for NULL by the OTA agent before this
 * function is called.
 * The file pointer/handle pFileContext->pFile, is checked for NULL by the OTA agent before this
 * function is called.
 * pData is checked for NULL by the OTA agent before this function is called.
 * blockSize is validated for range by the OTA agent before this function is called.
 * offset is validated by the OTA agent before this function is called.
 *
 * @param[in] pFileContext OTA file context information.
 * @param[in] ulOffset Byte offset to write to from the beginning of the file.
 * @param[in] pData Pointer to the byte array of data to write.
 * @param[in] ulBlockSize The number of bytes to write.
 *
 * @return The number of bytes written successfully, or a negative error code from the platform
 * abstraction layer.
 */
int16_t otaPal_WriteBlock( OtaFileContext_t * 	const pFileContext,
                           uint32_t 			ulOffset,
                           uint8_t * 			const pcData,
                           uint32_t 			ulBlockSize )
{

    uint32_t 	addr    = MRAM_ADDR;
    int32_t		ret     = 0;

    printf( "\n OTA PAL WriteBlock to MRAM \n");
    printf( "Address : 0x%X Offset: %u  BlockSize: %d \n", addr, ulOffset, ulBlockSize );

    if( (pFileContext == NULL) || (pFileContext != pxSystemContext ) )
    {
        return -1;
    }

    //Write to MRAM
    ret = FLASH_MRAM_drv->ProgramData(addr + ulOffset, pcData, ulBlockSize);

    return ret;
}


/**
 * @brief Activate the newest MCU image received via OTA.
 *
 * This function shall take necessary actions to activate the newest MCU
 * firmware received via OTA. It is typically just a reset of the device.
 *
 * @note This function SHOULD NOT return. If it does, the platform does not support
 * an automatic reset or an error occurred.
 *
 * @param[in] pFileContext OTA file context information.
 *
 * @return The OtaPalStatus_t error code is a combination of the main OTA PAL interface error and
 *         the MCU specific sub error code. See ota_platform_interface.h for the OtaPalMainStatus_t
 *         error codes and your specific PAL implementation for the sub error code.
 *
 * Major error codes returned are:
 *
 *   OtaPalSuccess on success.
 *   OtaPalActivateFailed: The activation of the new OTA image failed.
 */
OtaPalStatus_t otaPal_ActivateNewImage( OtaFileContext_t * const pFileContext )
{
	uint32_t    addr    = MRAM_ADDR;   //0x80100000
	uint32_t    eCode   = 0;
	uint32_t    sHandle = 0;
	uint32_t    retValue = 0;
	uint32_t    randValue = 0;
	uint8_t     revision_data[80] = {0};

	printf( "OTA PAL ActivateNewImage \n" );

        SE_Service_MHU_Reset_SoC();

	return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}

 //State;
/**
 * @brief Attempt to set the state of the OTA update image.
 *
 * Take required actions on the platform to Accept/Reject the OTA update image (or bundle).
 * Refer to the PAL implementation to determine what happens on your platform.
 *
 * @param[in] pFileContext File context of type OtaFileContext_t.
 * @param[in] eState The desired state of the OTA update image.
 *
 * @return The OtaPalStatus_t error code is a combination of the main OTA PAL interface error and
 *         the MCU specific sub error code. See ota_platform_interface.h for the OtaPalMainStatus_t
 *         error codes and your specific PAL implementation for the sub error code.
 *
 * Major error codes returned are:
 *
 *   OtaPalSuccess on success.
 *   OtaPalBadImageState: if you specify an invalid OtaImageState_t. No sub error code.
 *   OtaPalAbortFailed: failed to roll back the update image as requested by OtaImageStateAborted.
 *   OtaPalRejectFailed: failed to roll back the update image as requested by OtaImageStateRejected.
 *   OtaPalCommitFailed: failed to make the update image permanent as requested by OtaImageStateAccepted.
 */
OtaPalStatus_t otaPal_SetPlatformImageState( OtaFileContext_t * const pFileContext,
                                             OtaImageState_t eState )
{

    (void) eState;
	printf( "\nOTA PAL SetPlatformImage State. Set State :: [%d] \n", eState );

    if( pxSystemContext == NULL )
    {
        /* In this case, a reboot should have happened. */
        switch ( eState )
        {
            case OtaImageStateAccepted:
                break;
            case OtaImageStateRejected:
                /* The image is not the running image, the image in the secondary slot will be ereased if
                 * it is not a valid image. */
                break;
            case OtaImageStateTesting:
                RESTARTED = 1; //Mock TESTING Success
                break;
            case OtaImageStateAborted:
                /* The image download has been finished or has not been started.*/
                break;
            default:
                return OTA_PAL_COMBINE_ERR( OtaPalBadImageState, 0 );
        }
    }
    else
    {
        if( eState == OtaImageStateAccepted )
        {
            /* The image can only be set as accepted after a reboot. So the pxSystemContext should be NULL. */
            return OTA_PAL_COMBINE_ERR( OtaPalCommitFailed, 0 );
        }

        /* The image is still downloading and the OTA process will not continue. The image is in
         * the secondary slot and does not impact the later update process. So nothing to do in
         * other state.
         */
    }

    return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}

/**
 * @brief Get the state of the OTA update image.
 *
 * We read this at OTA_Init time and when the latest OTA job reports itself in self
 * test. If the update image is in the "pending commit" state, we start a self test
 * timer to assure that we can successfully connect to the OTA services and accept
 * the OTA update image within a reasonable amount of time (user configurable). If
 * we don't satisfy that requirement, we assume there is something wrong with the
 * firmware and automatically reset the device, causing it to roll back to the
 * previously known working code.
 *
 * If the update image state is not in "pending commit," the self test timer is
 * not started.
 *
 * @param[in] pFileContext File context of type OtaFileContext_t.
 *
 * @return An OtaPalImageState_t. One of the following:
 *   OtaPalImageStatePendingCommit (the new firmware image is in the self test phase)
 *   OtaPalImageStateValid         (the new firmware image is already committed)
 *   OtaPalImageStateInvalid       (the new firmware image is invalid or non-existent)
 *
 *   NOTE: OtaPalImageStateUnknown should NEVER be returned and indicates an implementation error.
 */
OtaPalImageState_t otaPal_GetPlatformImageState( OtaFileContext_t * const pFileContext )
{
    uint8_t ucSlot;

    printf( "OTA PAL GetPlatformImage State\n" );

    if (RESTARTED == 1){  //Mock Testing progress.
        RESTARTED = 0;
        return OtaPalImageStatePendingCommit;
    }

#if 0
    switch ( xImageInfo.state )
    {
        case PSA_IMAGE_PENDING_INSTALL:
            return OtaPalImageStatePendingCommit;
        case PSA_IMAGE_INSTALLED:
            return OtaPalImageStateValid;
        default:
            return OtaPalImageStateInvalid;
    }
#endif

    /* It should never goes here. But just for coding safety. */
    return OtaPalImageStateValid;
}

/**
 * @brief Reset the device.
 *
 * This function shall reset the MCU and cause a reboot of the system.
 *
 * @note This function SHOULD NOT return. If it does, the platform does not support
 * an automatic reset or an error occurred.
 *
 * @param[in] pFileContext OTA file context information.
 *
 * @return The OtaPalStatus_t error code is a combination of the main OTA PAL interface error and
 *         the MCU specific sub error code. See ota_platform_interface.h for the OtaPalMainStatus_t
 *         error codes and your specific PAL implementation for the sub error code.
 */
OtaPalStatus_t otaPal_ResetDevice( OtaFileContext_t * const pFileContext )
{
    //TODO Verify image code signature.
    printf( "OTA PAL ResetDevice... \n" );

    SE_Service_MHU_Reset_SoC();
    return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}
