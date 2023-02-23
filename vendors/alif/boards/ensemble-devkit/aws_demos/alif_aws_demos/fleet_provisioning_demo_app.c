/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */
/*
 * FreeRTOS V202203.00
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 */
/******************************************************************************
 * @file     fleet_provisioning_demo_app.c
 * @author   Sruthi Alajangi
 * @email    sruthi.alajangi@alifsemi.com
 * @version  V1.0.0
 * @date
 * @brief    File that demonstrates Fleet provisioning by claim feature of AWS.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Standard includes. */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo Config */
#include "fleet_provisioning_config.h"

/* mbedTLS include for configuring threading functions */
#include "mbedtls/threading.h"
#include "threading_alt.h"

/* TinyCBOR library for CBOR encoding and decoding operations. */
#include "cbor.h"

/* corePKCS11 includes. */
#include "core_pkcs11.h"
#include "core_pkcs11_config.h"

/* AWS IoT Fleet Provisioning Library. */
#include "fleet_provisioning.h"

/* PKCS11 & tinycbor helper includes */
#include "pkcs11_operations.h"
#include "mqtt_demo_helpers.h"
#include "tinycbor_serializer.h"

#define democonfigPROVISIONING_TEMPLATE_NAME "fleet_provisioning_template"

/**
 * @brief The length of #democonfigPROVISIONING_TEMPLATE_NAME.
 */
#define fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH    ( ( uint16_t ) ( sizeof( democonfigPROVISIONING_TEMPLATE_NAME ) - 1 ) )

/**
 * @brief The length of #democonfigFP_DEMO_ID.
 */
#define fpdemoFP_DEMO_ID_LENGTH                    ( ( uint16_t ) ( sizeof( democonfigFP_DEMO_ID ) - 1 ) )

/**
 * @brief Size of AWS IoT Thing name buffer.
 *
 * See https://docs.aws.amazon.com/iot/latest/apireference/API_CreateThing.html#iot-CreateThing-request-thingName
 */
#define fpdemoMAX_THING_NAME_LENGTH                128

/**
 * @brief The maximum number of times to run the loop in this demo.
 *
 * @note The demo loop is attempted to re-run only if it fails in an iteration.
 * Once the demo loop succeeds in an iteration, the demo exits successfully.
 */
#ifndef fpdemoMAX_DEMO_LOOP_COUNT
    #define fpdemoMAX_DEMO_LOOP_COUNT    ( 3 )
#endif

/**
 * @brief Time in seconds to wait between retries of the demo loop if
 * demo loop fails.
 */
#define fpdemoDELAY_BETWEEN_DEMO_RETRY_ITERATIONS_SECONDS    ( 5 )

/**
 * @brief Size of buffer in which to hold the private key.
 */
#define fpdemoPRIVATEKEY_BUFFER_LENGTH                              2048

/**
 * @brief Size of buffer in which to hold the certificate.
 */
#define fpdemoCERT_BUFFER_LENGTH                             2048

/**
 * @brief Size of buffer in which to hold the certificate id.
 *
 * See https://docs.aws.amazon.com/iot/latest/apireference/API_Certificate.html#iot-Type-Certificate-certificateId
 */
#define fpdemoCERT_ID_BUFFER_LENGTH                          64

/**
 * @brief Size of buffer in which to hold the certificate ownership token.
 */
#define fpdemoOWNERSHIP_TOKEN_BUFFER_LENGTH                  512

/**
 * @brief Milliseconds per second.
 */
#define fpdemoMILLISECONDS_PER_SECOND                        ( 1000U )

/**
 * @brief Milliseconds per FreeRTOS tick.
 */
#define fpdemoMILLISECONDS_PER_TICK                          ( fpdemoMILLISECONDS_PER_SECOND / configTICK_RATE_HZ )

/**
 * @brief Maximum number of outgoing publishes maintained in the application
 * until an ack is received from the broker.
 */
#define MAX_OUTGOING_PUBLISHES                       ( 1U )

/**
 * @brief Status values of the Fleet Provisioning response.
 */
typedef enum
{
    ResponseNotReceived,
    ResponseAccepted,
    ResponseRejected
} ResponseStatus_t;

/*-----------------------------------------------------------*/

/**
 * @brief Status reported from the MQTT publish callback.
 */
static ResponseStatus_t xResponseStatus;

/**
 * @brief Buffer to hold the provisioned AWS IoT Thing name.
 */
static char pcThingName[ fpdemoMAX_THING_NAME_LENGTH ];

/**
 * @brief Length of the AWS IoT Thing name.
 */
static size_t xThingNameLength;

/**
 * @brief Buffer to hold responses received from the AWS IoT Fleet Provisioning
 * APIs. When the MQTT publish callback receives an expected Fleet Provisioning
 * accepted payload, it copies it into this buffer.
 */
static uint8_t pucPayloadBuffer[ democonfigNETWORK_BUFFER_SIZE ];

/**
 * @brief Length of the payload stored in #pucPayloadBuffer. This is set by the
 * MQTT publish callback when it copies a received payload into #pucPayloadBuffer.
 */
static size_t xPayloadLength;

/**
 * @brief Each compilation unit that consumes the NetworkContext must define it.
 * It should contain a single pointer to the type of your desired transport.
 * When using multiple transports in the same compilation unit, define this pointer as void *.
 *
 * @note Transport stacks are defined in amazon-freertos/libraries/abstractions/transport/secure_sockets/transport_secure_sockets.h.
 */
struct NetworkContext
{
    SecureSocketsTransportParams_t * pParams;
};

/**
 * @brief Structure to keep the MQTT publish packets until an ack is received
 * for QoS1 publishes.
 */
typedef struct PublishPackets
{
    /**
     * @brief Packet identifier of the publish packet.
     */
    uint16_t packetId;

    /**
     * @brief Publish info of the publish packet.
     */
    MQTTPublishInfo_t pubInfo;
} PublishPackets_t;


/**
 * @brief The MQTT context used for MQTT operation.
 */
static MQTTContext_t xMqttContext;

/**
 * @brief The network context used for TLS operation.
 */
static NetworkContext_t xNetworkContext;

/**
 * @brief Global entry time into the application to use as a reference timestamp
 * in the #prvGetTimeMs function. #prvGetTimeMs will always return the difference
 * between the current time and the global entry time. This will reduce the chances
 * of overflow for the 32 bit unsigned integer used for holding the timestamp.
 */
static uint32_t ulGlobalEntryTimeMs;

/**
 * @brief Packet Identifier generated when Subscribe request was sent to the broker.
 *
 * It is used to match received Subscribe ACK to the transmitted subscribe
 * request.
 */
static uint16_t usGlobalSubscribePacketIdentifier = 0U;

/**
 * @brief Packet Identifier generated when Unsubscribe request was sent to the broker.
 *
 * It is used to match received Unsubscribe ACK to the transmitted unsubscribe
 * request.
 */
static uint16_t usGlobalUnsubscribePacketIdentifier = 0U;
/**
 *
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucSharedBuffer[ democonfigNETWORK_BUFFER_SIZE ];

/**
 * @brief Array to keep the outgoing publish messages.
 * These stored outgoing publish messages are kept until a successful ack
 * is received.
 */
static PublishPackets_t outgoingPublishPackets[ MAX_OUTGOING_PUBLISHES ] = { 0 };

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static MQTTFixedBuffer_t xBuffer =
{
    .pBuffer = ucSharedBuffer,
    .size    = democonfigNETWORK_BUFFER_SIZE
};
/*-----------------------------------------------------------*/

/**
 * @brief Callback to receive the incoming publish messages from the MQTT
 * broker. Sets xResponseStatus if an expected CreateKeysAndCertificate or
 * RegisterThing response is received, and copies the response into
 * responseBuffer if the response is an accepted one.
 *
 * @param[in] pPublishInfo Pointer to publish info of the incoming publish.
 * @param[in] usPacketIdentifier Packet identifier of the incoming publish.
 */
static void prvProvisioningPublishCallback( MQTTPublishInfo_t * pPublishInfo,
                                            uint16_t usPacketIdentifier );

/**
 * @brief Run the MQTT process loop to get a response.
 */
static bool prvWaitForResponse( void );

/**
 * @brief Subscribe to the CreateKeysAndCertificate accepted and rejected topics.
 */
static bool prvSubscribeToCreateKeysResponseTopics( void );

/**
 * @brief Unsubscribe from the CreateKeysAndCertificate accepted and rejected topics.
 */
static bool prvUnsubscribeFromCreateKeysResponseTopics( void );

/**
 * @brief Subscribe to the RegisterThing accepted and rejected topics.
 */
static bool prvSubscribeToRegisterThingResponseTopics( void );

/**
 * @brief Unsubscribe from the RegisterThing accepted and rejected topics.
 */
static bool prvUnsubscribeFromRegisterThingResponseTopics( void );

/**
 * @brief Function to clean up the publish packet with the given packet id.
 *
 * @param[in] usPacketId Packet identifier of the packet to be cleaned up from
 * the array.
 */
static void prvCleanupOutgoingPublishWithPacketID( uint16_t usPacketId );

static uint16_t usPublishPacketIdentifier;

#define mqttexampleTOPIC                                  "fleet_provisioning/telemetry/test"

#define fleetTOPIC  "$aws/certificates/create/cbor/accepted"

/*function to publish sample message to AWS IoT core on mqttexampleTOPIC*/
static BaseType_t prvMQTTPublishToTopic( MQTTContext_t * pxMQTTContext );

/**
 * @brief Context passed to tinyCBOR for #cborPrinter. Initial
 * state should be zeroed.
 */
typedef struct
{
    const char * str;
    size_t length;
} CborPrintContext_t;

/**
 * @brief Printing function to pass to tinyCBOR.
 *
 * cbor_value_to_pretty_stream calls it multiple times to print a textual CBOR
 * representation.
 *
 * @param token Context for the function.
 * @param fmt Printf style format string.
 * @param ... Printf style args after format string.
 */
static CborError cborPrinter( void * token,
                              const char * fmt,
                              ... );

const char * getStringFromCbor( const uint8_t * cbor,
                                size_t length )
{
    CborPrintContext_t printCtx = { 0 };
    CborParser parser;
    CborValue value;
    CborError error;

    error = cbor_parser_init( cbor, length, 0, &parser, &value );

    if( error == CborNoError )
    {
        error = cbor_value_to_pretty_stream( cborPrinter, &printCtx, &value, CborPrettyDefaultFlags );
    }

    if( error != CborNoError )
    {
        printf( "Error printing CBOR payload.\n" );
        printCtx.str = "";
    }

    return printCtx.str;
}

static void prvProvisioningPublishCallback( MQTTPublishInfo_t * pPublishInfo,
                                            uint16_t usPacketIdentifier )
{
    FleetProvisioningStatus_t status;
    FleetProvisioningTopic_t api;
    const char * cborDump;
    /* Silence compiler warnings about unused variables. */
    ( void ) usPacketIdentifier;

    status = FleetProvisioning_MatchTopic( pPublishInfo->pTopicName,
                                           pPublishInfo->topicNameLength, &api );

    if( status != FleetProvisioningSuccess )
    {
        printf( "Unexpected publish message received. Topic: %.*s.\n",
                ( int ) pPublishInfo->topicNameLength,
                ( const char * ) pPublishInfo->pTopicName);
    }
    else
    {
        if( api == FleetProvCborCreateKeysAndCertAccepted )
        {
            printf( "Received accepted response from Fleet Provisioning CreateKeysAndCert API.\n" );
            xResponseStatus = ResponseAccepted;
            /* Copy the payload from the MQTT library's buffer to #pucPayloadBuffer. */
            ( void ) memcpy( ( void * ) pucPayloadBuffer,
                             ( const void * ) pPublishInfo->pPayload,
                             ( size_t ) pPublishInfo->payloadLength );
            xPayloadLength = pPublishInfo->payloadLength;
            cborDump = getStringFromCbor( ( const uint8_t * ) pPublishInfo->pPayload, pPublishInfo->payloadLength );
            printf( "accepted response from Fleet Provisioning CreateKeysAndCert API is %s\n",cborDump );
            free( ( void * ) cborDump );
        }
        else if( api == FleetProvCborCreateKeysAndCertRejected )
        {
            printf( "Received rejected response from Fleet Provisioning CreateKeysAndCert API.\n" );
            xResponseStatus = ResponseRejected;
            cborDump = getStringFromCbor( ( const uint8_t * ) pPublishInfo->pPayload, pPublishInfo->payloadLength );
            printf("rejected response from Fleet Provisioning CreateKeysAndCert API received is %s\n",cborDump );
            free( ( void * ) cborDump );
        }
        else if( api == FleetProvCborRegisterThingAccepted )
        {
            printf( "Received accepted response from Fleet Provisioning RegisterThing API.\n" );
            xResponseStatus = ResponseAccepted;
            /* Copy the payload from the MQTT library's buffer to #pucPayloadBuffer. */
            ( void ) memcpy( ( void * ) pucPayloadBuffer,
                             ( const void * ) pPublishInfo->pPayload,
                             ( size_t ) pPublishInfo->payloadLength );
            xPayloadLength = pPublishInfo->payloadLength;
            cborDump = getStringFromCbor( ( const uint8_t * ) pPublishInfo->pPayload, pPublishInfo->payloadLength );
            printf( "accepted response from Fleet Provisioning RegisterThing API received is %s\n",cborDump );
            free( ( void * ) cborDump );
        }
        else if( api == FleetProvCborRegisterThingRejected )
        {
            printf( "Received rejected response from Fleet Provisioning RegisterThing API.\n" );
            xResponseStatus = ResponseRejected;
        }
        else
        {
            printf( "Received message on unexpected Fleet Provisioning topic. Topic: %.*s.\n",
                    ( int ) pPublishInfo->topicNameLength,
                    ( const char * ) pPublishInfo->pTopicName  );
        }
    }
}

static bool prvWaitForResponse( void )
{
    bool xStatus = false;
    bool processLoop_status;
    xResponseStatus = ResponseNotReceived;

    /* xResponseStatus is updated from the MQTT publish callback. */
    processLoop_status = ProcessLoop( &xMqttContext, 5000 );

    if( xResponseStatus == ResponseNotReceived )
    {
        printf("Timed out waiting for response.\n");
    }

    if( xResponseStatus == ResponseAccepted )
    {
        xStatus = true;
    }
    return xStatus;
}

static bool prvSubscribeToCreateKeysResponseTopics( void )
{
    bool xStatus;
    xStatus = SubscribeToTopic( &xMqttContext,
                                FP_CBOR_CREATE_KEYS_ACCEPTED_TOPIC,
                                FP_CBOR_CREATE_KEYS_ACCEPTED_LENGTH );

    if( xStatus == false )
    {
        printf( "Failed to subscribe to fleet provisioning topic: %.*s.",
        		FP_CBOR_CREATE_KEYS_ACCEPTED_LENGTH,
                FP_CBOR_CREATE_KEYS_ACCEPTED_TOPIC);
    }
    if( xStatus == true )
    {
        printf("subscribed to %s\n",fleetTOPIC);
    }
    if( xStatus == true )
    {
        xStatus = SubscribeToTopic( &xMqttContext,
                                    FP_CBOR_CREATE_KEYS_REJECTED_TOPIC,
                                    FP_CBOR_CREATE_KEYS_REJECTED_LENGTH );
        if( xStatus == false )
        {
            printf( "Failed to subscribe to fleet provisioning topic: %.*s.",
                    FP_CBOR_CREATE_KEYS_REJECTED_LENGTH,
                    FP_CBOR_CREATE_KEYS_REJECTED_TOPIC );
        }
        if( xStatus == true )
        {
            printf("subscribed to %s\n",FP_CBOR_CREATE_KEYS_REJECTED_TOPIC);
        }
    }
    return xStatus;
}

static bool prvUnsubscribeFromCreateKeysResponseTopics( void )
{
    bool xStatus;
    xStatus = UnsubscribeFromTopic( &xMqttContext,
                                    FP_CBOR_CREATE_KEYS_ACCEPTED_TOPIC,
                                    FP_CBOR_CREATE_KEYS_ACCEPTED_LENGTH );
    if( xStatus == false )
    {
        printf( "Failed to unsubscribe from fleet provisioning topic: %.*s.",
                FP_CBOR_CREATE_KEYS_ACCEPTED_LENGTH,
                FP_CBOR_CREATE_KEYS_ACCEPTED_TOPIC);
    }
    if( xStatus == true )
    {
        xStatus = UnsubscribeFromTopic( &xMqttContext,
                                        FP_CBOR_CREATE_KEYS_REJECTED_TOPIC,
                                        FP_CBOR_CREATE_KEYS_REJECTED_LENGTH );
        if( xStatus == false )
        {
            printf( "Failed to unsubscribe from fleet provisioning topic: %.*s.",
                    FP_CBOR_CREATE_KEYS_REJECTED_LENGTH,
                    FP_CBOR_CREATE_KEYS_REJECTED_TOPIC );
        }
    }
    return xStatus;
}

static bool prvSubscribeToRegisterThingResponseTopics( void )
{
    bool xStatus;
    xStatus = SubscribeToTopic( &xMqttContext,
                                FP_CBOR_REGISTER_ACCEPTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ),
                                FP_CBOR_REGISTER_ACCEPTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ) );

    if( xStatus == false )
    {
        printf( "Failed to subscribe to fleet provisioning topic: %.*s.",
                FP_CBOR_REGISTER_ACCEPTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ),
                FP_CBOR_REGISTER_ACCEPTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ) );
    }
    if( xStatus == true )
    {
        xStatus = SubscribeToTopic( &xMqttContext,
                                    FP_CBOR_REGISTER_REJECTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ),
                                    FP_CBOR_REGISTER_REJECTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ) );
        if( xStatus == false )
        {
            printf( "Failed to subscribe to fleet provisioning topic: %.*s.",
                    FP_CBOR_REGISTER_REJECTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ),
                    FP_CBOR_REGISTER_REJECTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ) );
        }
    }
    return xStatus;
}

static bool prvUnsubscribeFromRegisterThingResponseTopics( void )
{
    bool xStatus;
    xStatus = UnsubscribeFromTopic( &xMqttContext,
                                    FP_CBOR_REGISTER_ACCEPTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ),
                                    FP_CBOR_REGISTER_ACCEPTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ) );
    if( xStatus == false )
    {
        printf( "Failed to unsubscribe from fleet provisioning topic: %.*s.",
                FP_CBOR_REGISTER_ACCEPTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ),
                FP_CBOR_REGISTER_ACCEPTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ) );
    }
    if( xStatus == true )
    {
        xStatus = UnsubscribeFromTopic( &xMqttContext,
                                        FP_CBOR_REGISTER_REJECTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ),
                                        FP_CBOR_REGISTER_REJECTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ) );
        if( xStatus == false )
        {
            printf( "Failed to unsubscribe from fleet provisioning topic: %.*s.",
                    FP_CBOR_REGISTER_REJECTED_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ),
                    FP_CBOR_REGISTER_REJECTED_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ) );
        }
    }
    return xStatus;
}

static void prvMqttCallback (MQTTContext_t * pxMqttContext,
                             MQTTPacketInfo_t * pxPacketInfo,
                             MQTTDeserializedInfo_t * pxDeserializedInfo)
{
    uint16_t usPacketIdentifier;
    configASSERT( pxMqttContext != NULL );
    configASSERT( pxPacketInfo != NULL );
    configASSERT( pxDeserializedInfo != NULL );
    /* Suppress the unused parameter warning when asserts are disabled in build. */
    ( void ) pxMqttContext;
    usPacketIdentifier = pxDeserializedInfo->packetIdentifier;

    /* Handle an incoming publish. The lower 4 bits of the publish packet
     * type is used for the dup, QoS, and retain flags. Hence masking
     * out the lower bits to check if the packet is publish. */
    if( ( pxPacketInfo->type & 0xF0U ) == MQTT_PACKET_TYPE_PUBLISH )
    {
        printf("incoming publish message received\n");
        configASSERT( pxDeserializedInfo->pPublishInfo != NULL );
        /* Invoke the application callback for incoming publishes. */
        prvProvisioningPublishCallback( pxDeserializedInfo->pPublishInfo, usPacketIdentifier );
    }
    else
    {
        vHandleOtherIncomingPacket( pxPacketInfo, usPacketIdentifier );
    }
}

static void prvCleanupOutgoingPublishWithPacketID( uint16_t usPacketId )
{
    uint8_t ucIndex = 0;
    assert( outgoingPublishPackets != NULL );
    assert( usPacketId != MQTT_PACKET_ID_INVALID );

    /* Clean up all the saved outgoing publishes. */
    for( ; ucIndex < MAX_OUTGOING_PUBLISHES; ucIndex++ )
    {
        if( outgoingPublishPackets[ ucIndex ].packetId == usPacketId )
        {
            vCleanupOutgoingPublishAt( ucIndex );
            printf( "Cleaned up outgoing publish packet with packet id %u.\n\n",
                    usPacketId );
            break;
        }
    }
}

int RunFleetProvisioningDemo(void * pvParameters)
{
    bool xStatus = true;
    /* Buffer for holding the key. */
    char pcPrivateKey[ fpdemoPRIVATEKEY_BUFFER_LENGTH ] = { 0 };
    size_t xPrivateKeyLength;
    /* Buffer for holding received certificate until it is saved. */
    char pcCertificate[ fpdemoCERT_BUFFER_LENGTH ];
    size_t xCertificateLength;
    /* Buffer for holding the certificate ID. */
    char pcCertificateId[ fpdemoCERT_ID_BUFFER_LENGTH ];
    size_t xCertificateIdLength;
    /* Buffer for holding the certificate ownership token. */
    char pcOwnershipToken[ fpdemoOWNERSHIP_TOKEN_BUFFER_LENGTH ];
    size_t xOwnershipTokenLength;
    bool xConnectionEstablished = false;
    CK_SESSION_HANDLE xP11Session;
    uint32_t ulDemoRunCount = 0U;
    CK_RV xPkcs11Ret = CKR_OK;
    char *str1 = "from CreateKeysAndCertificate";
    bool claim_publish_Status = true;
    /* Silence compiler warnings about unused variables. */
    ( void ) pvParameters;
    do
    {
        /* Initialize the buffer lengths to their max lengths. */
        xPrivateKeyLength = fpdemoPRIVATEKEY_BUFFER_LENGTH;
        xCertificateLength = fpdemoCERT_BUFFER_LENGTH;
        xCertificateIdLength = fpdemoCERT_ID_BUFFER_LENGTH;
        xOwnershipTokenLength = fpdemoOWNERSHIP_TOKEN_BUFFER_LENGTH;
        /**** Connect to AWS IoT Core with provisioning claim credentials *****/
        /* We first use the claim credentials to connect to the broker. These
         * credentials should allow use of the RegisterThing API and one of the
         * CreateCertificatefromCsr or CreateKeysAndCertificate.
         * In this demo we use CreateKeysAndCertificate. */
        if( xStatus == true )
        {
            /* Attempts to connect to the AWS IoT MQTT broker. If the
             * connection fails, retries after a timeout. Timeout value will
             * exponentially increase until maximum attempts are reached. */
            printf("Establishing MQTT session with claim certificate...\n" );
            xStatus = EstablishMqttSession( &xMqttContext,
                                            &xNetworkContext,
                                            &xBuffer,
                                            prvMqttCallback );
            if( xStatus == false )
            {
                printf("Failed to establish MQTT session\n");
            }
            else
            {
                printf("Established connection with claim credentials.\n" );
                xConnectionEstablished = true;
            }
        }
        /**** Call the CreateKeysAndCertificate API ***************************/
        /* We use the CreateKeysAndCertificate API to obtain a client certificate and private key */
        if( xStatus == true )
        {
            /* Subscribe to the CreateKeysAndCertificate accepted and rejected
             * topics. In this demo we use CBOR encoding for the payloads,
             * so we use the CBOR variants of the topics. */
            xStatus = prvSubscribeToCreateKeysResponseTopics();
        }
        if( xStatus == true )
        {
            /* Publish empty message payload to the CreateKeysAndCertificate API. */
            PublishToTopic( &xMqttContext,
                            FP_CBOR_CREATE_KEYS_PUBLISH_TOPIC,
                            FP_CBOR_CREATE_KEYS_PUBLISH_LENGTH,
                            NULL,
                            0);
            if( xStatus == false )
            {
                printf( "Failed to publish to fleet provisioning topic: %.*s.\n",
                        FP_CBOR_CREATE_CERT_PUBLISH_LENGTH,
                        FP_CBOR_CREATE_CERT_PUBLISH_TOPIC);
            }
        }
        if( xStatus == true )
        {
            /* Get the response to the CreateKeysAndCertificate request. */
            xStatus = prvWaitForResponse();
        }
        if( xStatus == true )
        {
            /* From the response, extract the private key, certificate, certificate ID, and
             * certificate ownership token. */
            xStatus = xParseResponse( pucPayloadBuffer,
                                         xPayloadLength,
                                         pcPrivateKey,
                                         xPrivateKeyLength,
                                         pcCertificate,
                                         &xCertificateLength,
                                         pcCertificateId,
                                         &xCertificateIdLength,
                                         pcOwnershipToken,
                                         &xOwnershipTokenLength );
        }
        if( xStatus == true )
        {
            /* Save the certificate into PKCS #11. */
            xStatus = xLoadCertificate( xP11Session,
                                        pcCertificate,
                                        pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,
                                        xCertificateLength  );
        }

        if( xStatus == true )
        {
            /* Save the privateKey into PKCS #11. */
            xStatus = xLoadPrivateKey( xP11Session,
                                       pcPrivateKey,
                                       pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,
                                       xPrivateKeyLength);
        }


        if( xStatus == true )
        {
            /* Unsubscribe from the CreateKeysAndCertificate topics. */
            xStatus = prvUnsubscribeFromCreateKeysResponseTopics();
        }
        /**** Call the RegisterThing API **************************************/
        /* We then use the RegisterThing API to activate the received certificate,
         * provision AWS IoT resources according to the provisioning template, and
         * receive device configuration. */
        if( xStatus == true )
        {
            /* Create the request payload to publish to the RegisterThing API. */
            xStatus = xGenerateRegisterThingRequest( pucPayloadBuffer,
                                                     democonfigNETWORK_BUFFER_SIZE,
                                                     pcOwnershipToken,
                                                     xOwnershipTokenLength,
                                                     democonfigFP_DEMO_ID,
                                                     fpdemoFP_DEMO_ID_LENGTH,
                                                     &xPayloadLength );
        }
        if( xStatus == true )
        {
            /* Subscribe to the RegisterThing response topics. */
            xStatus = prvSubscribeToRegisterThingResponseTopics();
        }
        if( xStatus == true )
        {
            /* Publish the RegisterThing request. */
            PublishToTopic( &xMqttContext,
                           FP_CBOR_REGISTER_PUBLISH_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ),
                           FP_CBOR_REGISTER_PUBLISH_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ),
                           ( char * ) pucPayloadBuffer,
                           xPayloadLength );
            if( xStatus == false )
            {
                printf( "Failed to publish to fleet provisioning topic: %.*s.",
                        FP_CBOR_REGISTER_PUBLISH_LENGTH( fpdemoPROVISIONING_TEMPLATE_NAME_LENGTH ),
                        FP_CBOR_REGISTER_PUBLISH_TOPIC( democonfigPROVISIONING_TEMPLATE_NAME ));
            }
        }
        if( xStatus == true )
        {
            /* Get the response to the RegisterThing request. */
            xStatus = prvWaitForResponse();
        }
        if( xStatus == true )
        {
            /* Extract the Thing name from the response. */
            xThingNameLength = fpdemoMAX_THING_NAME_LENGTH;
            xStatus = xParseRegisterThingResponse( pucPayloadBuffer,
                                                   xPayloadLength,
                                                   pcThingName,
                                                   &xThingNameLength );
            if( xStatus == true )
            {
                printf("Received AWS IoT Thing name: %.*s\n", ( int ) xThingNameLength, pcThingName);
            }
        }
        if( xStatus == true )
        {
            /* Unsubscribe from the RegisterThing topics. */
            prvUnsubscribeFromRegisterThingResponseTopics();
        }
        /**** Disconnect from AWS IoT Core ************************************/
        /* As we have completed the provisioning workflow, we disconnect from
         * the connection using the provisioning claim credentials. We will
         * establish a new MQTT connection with the newly provisioned credentials. */
        if( xConnectionEstablished == true )
        {
            printf("Disconnecting from broker\n");
            DisconnectMqttSession( &xMqttContext, &xNetworkContext );
            xConnectionEstablished = false;
        }
        /**** Connect to AWS IoT Core with provisioned certificate ************/
        if( xStatus == true )
        {
            printf( "Establishing MQTT session with provisioned certificate...\n" );
            xStatus = EstablishMqttSession( &xMqttContext,
                                            &xNetworkContext,
                                            &xBuffer,
                                            prvMqttCallback );
            if( xStatus != true )
            {   printf( "Failed to establish MQTT session with provisioned "
                        "credentials. Verify on your AWS account that the "
                        "new certificate is active and has an attached IoT "
                        "Policy that allows the \"iot:Connect\" action.\n" );
            }
            else
            {
                printf("Sucessfully established connection with provisioned credentials.\n");
                xConnectionEstablished = true;
                for(int i=0;i<10;i++)
                {
                    xStatus = prvMQTTPublishToTopic( &xMqttContext );
                    if( xStatus != true )
                    {
                        printf( "ERROR : Publishing Sample message failed\n" );
                    }
                }
                if( xConnectionEstablished == true )
                        {
                            printf("Disconnecting from broker\n");
                            DisconnectMqttSession( &xMqttContext, &xNetworkContext );
                            xConnectionEstablished = false;
                        }

            }
        }
    }while( xStatus != true );
    if( xStatus == true )
    {
        printf("Demo completed successfully.\n");
    }
    return ( xStatus == true ) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/*function to publish sample message to AWS IoT core on mqttexampleTOPIC*/
static BaseType_t prvMQTTPublishToTopic( MQTTContext_t * pxMQTTContext )
{
    MQTTStatus_t xResult;
    MQTTPublishInfo_t xMQTTPublishInfo;
    BaseType_t xStatus = pdPASS;
    char *str = "hello from FPDemoID_Device2";

   /* Some fields are not used by this demo so start with everything at 0. */
     ( void ) memset( ( void * ) &xMQTTPublishInfo, 0x00, sizeof( xMQTTPublishInfo ) );

     /* This demo uses QoS1. */
     xMQTTPublishInfo.qos = MQTTQoS1;
     xMQTTPublishInfo.retain = false;
     xMQTTPublishInfo.pTopicName = mqttexampleTOPIC;
     xMQTTPublishInfo.topicNameLength = ( uint16_t ) strlen( mqttexampleTOPIC );
     xMQTTPublishInfo.pPayload = str;
     xMQTTPublishInfo.payloadLength = strlen(str);

     /* Get a unique packet id. */
     usPublishPacketIdentifier = MQTT_GetPacketId( pxMQTTContext );

     /* Send PUBLISH packet. Packet ID is not used for a QoS1 publish. */
     xResult = MQTT_Publish( pxMQTTContext, &xMQTTPublishInfo, usPublishPacketIdentifier );

     if( xResult != MQTTSuccess )
     {
         xStatus = pdFAIL;
         printf( "ERROR : Failed to send PUBLISH message to broker: Topic=%s, Error=%s\n",mqttexampleTOPIC,MQTT_Status_strerror( xResult) );
     }

     return xStatus;
 }

static CborError cborPrinter( void * token,
                              const char * fmt,
                              ... )
{
    int result;
    va_list args;
    CborPrintContext_t * ctx = ( CborPrintContext_t * ) token;

    va_start( args, fmt );

    /* Compute length to write. */
    result = vsnprintf( NULL, 0, fmt, args );

    va_end( args );

    if( result < 0 )
    {
        printf( "Error formatting CBOR string.\n" );
    }
    else
    {
        size_t newLen = ( unsigned ) result;
        size_t oldLen = ctx->length;
        char * newPtr;

        ctx->length = oldLen + newLen;
        newPtr = ( char * ) realloc( ( void * ) ctx->str, ctx->length + 1 );

        if( newPtr == NULL )
        {
            printf( "Failed to reallocate CBOR string.\n" );
            result = -1;
        }
        else
        {
            va_start( args, fmt );

            result = vsnprintf( newPtr + oldLen, newLen + 1, fmt, args );

            va_end( args );

            ctx->str = newPtr;

            if( result < 0 )
            {
                printf( "Error printing CBOR string.\n" );
            }
        }
    }

    return ( result < 0 ) ? CborErrorIO : CborNoError;
}
