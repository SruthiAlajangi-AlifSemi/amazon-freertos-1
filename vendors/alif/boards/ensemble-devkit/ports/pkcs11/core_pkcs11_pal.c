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
 * @file     core_pkcs11_pal.c
 * @author   Sruthi Alajangi
 * @email    sruthi.alajangi@alifsemi.com
 * @version  V1.0.0
 * @date     21-Sep-2022
 * @brief    PKCS11 PAL for saving & getting client credentials required for TLS handshake between client and AWS IOT Core.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/


/* C runtime includes. */
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"

/* PKCS #11 includes. */
#include "core_pkcs11.h"
#include "core_pkcs11_config.h"

unsigned char* Cert_PTR;
unsigned char* Key_PTR;

CK_BYTE_PTR Cert_PTR1;
CK_BYTE_PTR Key_PTR1;

int CertSize;
int KeySize;


typedef struct _pkcs_data
{
    CK_ATTRIBUTE Label;
    uint32_t ulDataSize;
    CK_OBJECT_HANDLE xHandle;
}PKCS_DATA;

#define PKCS_HANDLES_LABEL_MAX_LENGTH 40
#define PKCS_OBJECT_HANDLES_NUM 5

enum eObjectHandles
{
    eInvalidHandle = 0, /* According to PKCS #11 spec, 0 is never a valid object handle. */
    eDevicePrivateKey = 1,
    eDevicePublicKey,
    eDeviceCertificate,
    eCodeSigningKey,
};

uint8_t object_handle_dictionary[PKCS_OBJECT_HANDLES_NUM][PKCS_HANDLES_LABEL_MAX_LENGTH] =
{
    pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,
    pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS,
    pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,
    pkcs11configLABEL_CODE_VERIFICATION_KEY,
};
void PAL_UTILS_LabelToHandle( const char * pcLabel,CK_OBJECT_HANDLE_PTR pHandle );

CK_RV PKCS11_PAL_Initialize( void )
{
    CK_RV xResult = CKR_OK;

    CRYPTO_Init();
    return xResult;
}

CK_OBJECT_HANDLE PKCS11_PAL_SaveObject( CK_ATTRIBUTE_PTR pxLabel,CK_BYTE_PTR pucData,CK_ULONG ulDataSize )
{
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;
    PAL_UTILS_LabelToHandle( pxLabel->pValue,&xHandle );
    if( 0 == strncmp((const char *) pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,(const char *)pxLabel->pValue,sizeof( pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS ) ) )
    {
        xHandle = eDeviceCertificate;
        if(Cert_PTR != NULL)
        {
            printf("Cert_PTR is not NULL");
        }
        Cert_PTR = pvPortMalloc(ulDataSize);
        if(Cert_PTR != NULL)
        {
            memcpy(Cert_PTR, pucData, ulDataSize);
        }
        CertSize = ulDataSize ;
    }
    else if( 0 == strncmp( (const char *) pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,(const char *) pxLabel->pValue,sizeof( pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS ) ) )
    {
        xHandle = eDevicePrivateKey;
        Key_PTR = pvPortMalloc(ulDataSize);
        if(Key_PTR != NULL)
        {
            memcpy(Key_PTR, pucData, ulDataSize);
        }
        KeySize = ulDataSize;
    }
	else
	{
	    xHandle =  eInvalidHandle;
	}
    return xHandle;
}

CK_OBJECT_HANDLE PKCS11_PAL_FindObject( CK_BYTE_PTR pxLabel,CK_ULONG usLength )
{
    ( void ) usLength;
    CK_OBJECT_HANDLE xHandle = 0;
    if( 0 == strcmp(pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,pxLabel) )
    {
        xHandle = eDeviceCertificate;
    }
    else if( 0 == strcmp(pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,pxLabel) )
    {
        xHandle = eDevicePrivateKey;
    }
    else
    {
        xHandle =  eInvalidHandle;
    }
    return xHandle;
}

CK_RV PKCS11_PAL_GetObjectValue( CK_OBJECT_HANDLE xHandle,CK_BYTE_PTR * ppucData,
                                 CK_ULONG_PTR pulDataSize,CK_BBOOL * pIsPrivate )
{
    CK_RV ulReturn = CKR_KEY_HANDLE_INVALID;
    CK_OBJECT_HANDLE xHandleStorage = xHandle;
    if (xHandle != eInvalidHandle)
    {
        if (xHandle == eDevicePrivateKey)
        {
            *ppucData = Key_PTR;
            *pulDataSize = KeySize;
            *pIsPrivate = CK_TRUE;
            ulReturn = CKR_OK;
        }
        if (xHandle == eDeviceCertificate)
        {
            *ppucData = Cert_PTR;
            *pulDataSize = CertSize;
            *pIsPrivate = CK_FALSE;
            ulReturn = CKR_OK;
        }
    }
    else
    {
        printf("GetObject:eInvalidHandle\n");
    }
    return ulReturn;
}

void PKCS11_PAL_GetObjectValueCleanup( CK_BYTE_PTR pucData,CK_ULONG ulDataSize )
{
    /* Unused parameters. */
    ( void ) pucData;
    ( void ) ulDataSize;
}

void prvHandleToLabel( char ** pcLabel,CK_OBJECT_HANDLE xHandle )
{
    if( pcLabel != NULL )
    {
        switch( xHandle )
        {
            case eDeviceCertificate:
            	*pcLabel = ( char * ) pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS;
                break;
            case eDevicePrivateKey:
                *pcLabel = ( char * ) pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS;
                break;
            case eDevicePublicKey:
                *pcLabel = ( char * ) pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS;
                break;
            case eCodeSigningKey:
                *pcLabel = ( char * ) pkcs11configLABEL_CODE_VERIFICATION_KEY;
                break;
            default:
                *pcLabel = NULL;
                 break;
        }
    }
}
/*-----------------------------------------------------------*/

CK_RV PKCS11_PAL_DestroyObject( CK_OBJECT_HANDLE xHandle )
{
    if (xHandle != eInvalidHandle)
    {
        return CKR_OK;
    }
    else
    {
        return xHandle;
    }
}

void PAL_UTILS_LabelToHandle( const char * pcLabel,CK_OBJECT_HANDLE_PTR pHandle )
{
    if( ( pcLabel != NULL ) && ( pHandle != NULL ) )
    {
        if( 0 == strncmp( pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,pcLabel,sizeof( pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS ) ) )
        {
            *pHandle = ( CK_OBJECT_HANDLE ) eDeviceCertificate;
        }
        else if( 0 == strncmp( pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,pcLabel,sizeof( pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS ) ) )
        {
            *pHandle = ( CK_OBJECT_HANDLE ) eDevicePrivateKey;
        }
        else if( 0 == strncmp( pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS,pcLabel,sizeof( pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS ) ) )
        {
            *pHandle = ( CK_OBJECT_HANDLE ) eDevicePublicKey;
        }
        else
        {
            *pHandle = ( CK_OBJECT_HANDLE ) eInvalidHandle;
        }
    }
    else
    {
        LogError( ( "Could not convert label to handle. Received a NULL parameter." ) );
    }
}
