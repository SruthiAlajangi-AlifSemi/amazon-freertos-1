/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
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
 * @file     shadow_demo_app.c
 * @author   Sruthi Alajangi
 * @email    sruthi.alajangi@alifsemi.com
 * @version  V1.0.0
 * @date
 * @brief    File that demonstrates usage of AWS IoT Device Shadow Service with use case of toggling LED.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/
/**
 * @file shadow_demo_app.c

 * In This demo led is initialized to powerOn state and does following operations.
 * 1. Establish a MQTT connection by using the helper functions in mqtt_demo_helpers.c.
 * 2. Assemble strings for the MQTT topics of device shadow, by using macros defined by the Device Shadow library.
 * 3. Subscribe to those MQTT topics by using helper functions in mqtt_demo_helpers.c.
 * 4. Publish a reported state of powerOn by using helper functions in mqtt_demo_helpers.c.
 * 5. Handle incoming MQTT messages in prvEventCallback, determine whether the message is related to the device
 * shadow by using a function defined by the Device Shadow library (Shadow_MatchTopic). If the message is a
 * device shadow delta message, change state of led as received in delta message
 * and then publish the reported state to AWS IoT device shadow.
 * 6. Handle incoming message again in prvEventCallback. If the message is from update/accepted, verify that it
 * has the same clientToken as previously published in the update message.
 * 7.Publish sample message to AWS IoT Core to keep connection alive between device and AWS IoT broker.
 *
 * @note This demo uses retry logic to connect to AWS IoT broker if connection attempts fail.
 * The FreeRTOS/backoffAlgorithm library is used to calculate the retry interval with an exponential
 * backoff and jitter algorithm. For generating random number required by the algorithm, the PKCS11
 * module is used as it allows access to a True Random Number Generator (TRNG) if the vendor platform
 * supports it.
 * It is RECOMMENDED to seed the random number generator with a device-specific entropy source so that
 * probability of collisions from devices in connection retries is mitigated.
 *
 */
/* Standard includes. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* GPIO driver includes */
#include "Driver_GPIO.h"

/* Demo Specific configs. */
#include "aws_demo.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* SHADOW API header. */
#include "shadow.h"

/* JSON library includes. */
#include "core_json.h"

/* shadow demo helpers header. */
#include "mqtt_demo_helpers.h"

/* Transport interface implementation include header for TLS. */
#include "transport_secure_sockets.h"

#include "shadow_demo_config.h"
/**
 * @brief Format string representing a Shadow document with a "reported" state.
 *
 * The real json document will look like this:
 * {
 *   "state": {
 *     "reported": {
 *       "powerOn": 1
 *     }
 *   },
 *   "clientToken": "021909"
 * }
 *
 * Note the client token, which is required for all Shadow updates. The client
 * token must be unique at any given time, but may be reused once the update is
 * completed. For this demo, a timestamp is used for a client token.
 */
//#define SHADOW_REPORTED_JSON    \
//    "{"                         \
//    "\"state\":{"               \
//    "\"reported\":{"            \
//    "\"powerOn\":%01d"          \
//    "}"                         \
//    "},"                        \
//    "\"clientToken\":\"%06lu\"" \
//    "}"
#define SHADOW_REPORTED_JSON    \
    "{"                         \
    "\"state\":{"               \
    "\"reported\":{"            \
    "\"Temperature\":%05f,"      \
	"\"Accelerometer\":%01d,"    \
	"\"Magnetometer\":%01d"     \
    "}"                         \
    "},"                        \
    "\"clientToken\":\"%06lu\"" \
    "}"
//#define SHADOW_REPORTED_JSON    \
//    "{"                         \
//    "\"state\":{"               \
//    "\"reported\":{"            \
//    "\"Temperature\":%05f"     \
//    "}"                         \
//    "},"                        \
//    "\"clientToken\":\"%06lu\"" \
//    "}"
/**
 * @brief The expected size of #SHADOW_REPORTED_JSON.
 *
 * Because all the format specifiers in #SHADOW_REPORTED_JSON include a length,
 * its full size is known at compile-time by pre-calculation. Users could refer to
 * the way how to calculate the actual length in #SHADOW_DESIRED_JSON_LENGTH.
 */
#define SHADOW_REPORTED_JSON_LENGTH    ( sizeof( SHADOW_REPORTED_JSON ) + 3)

#ifndef THING_NAME

/**
 * @brief Predefined thing name.
 *
 * This is the example predefine thing name and could be compiled in ROM code.
 */
    #define THING_NAME    democonfigCLIENT_IDENTIFIER
#endif

/**
 * @brief The length of #THING_NAME.
 */
#define THING_NAME_LENGTH    ( ( uint16_t ) ( sizeof( THING_NAME ) - 1 ) )

/**
 * @brief The maximum number of times to run the loop in this demo.
 */
#ifndef SHADOW_MAX_DEMO_COUNT
    #define SHADOW_MAX_DEMO_COUNT    ( 10 )
#endif

/**
 * @brief Time in ticks to wait between each cycle of the demo implemented
 * by RunDeviceShadowDemo().
 */
#define DELAY_BETWEEN_DEMO_ITERATIONS_TICKS             ( pdMS_TO_TICKS( 5000U ) )

/**
 * @brief The maximum number of times to call MQTT_ProcessLoop() when waiting
 * for a response for Shadow delete operation.
 */
#define MQTT_PROCESS_LOOP_DELETE_RESPONSE_COUNT_MAX     ( 30U )

/**
 * @brief Timeout for MQTT_ProcessLoop in milliseconds.
 */
#define MQTT_PROCESS_LOOP_TIMEOUT_MS                    ( 700U )

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

/*-----------------------------------------------------------*/


/**
 * @brief The MQTT context used for MQTT operation.
 */
static MQTTContext_t xMqttContext;

/**
 * @brief The network context used for TLS operation.
 */
static NetworkContext_t xNetworkContext;

/**
 *
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucSharedBuffer[ democonfigNETWORK_BUFFER_SIZE ];

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static MQTTFixedBuffer_t xBuffer =
{
    .pBuffer = ucSharedBuffer,
    .size    = democonfigNETWORK_BUFFER_SIZE
};

/**
 * @brief The simulated device current power on state.
 */
static uint32_t ulCurrentPowerOnState = 1U;

/**
 * @brief The flag to indicate the device current power on state changed.
 */
static bool stateChanged = false;

/**
 * @brief When we send an update to the device shadow, and if we care about
 * the response from cloud (accepted/rejected), remember the clientToken and
 * use it to match with the response.
 */
static uint32_t ulClientToken = 0U;

/**
 * @brief The return status of prvUpdateDeltaHandler callback function.
 */
static BaseType_t xUpdateDeltaReturn = pdPASS;

/**
 * @brief The return status of prvUpdateAcceptedHandler callback function.
 */
static BaseType_t xUpdateAcceptedReturn = pdPASS;

float temperature;

/*-----------------------------------------------------------*/

/**
 * @brief This example uses the MQTT library of the AWS IoT Device SDK for
 * Embedded C. This is the prototype of the callback function defined by
 * that library. It will be invoked whenever the MQTT library receives an
 * incoming message.
 *
 * @param[in] pxMqttContext MQTT context pointer.
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] pxDeserializedInfo Deserialized information from the incoming packet.
 */
static void prvEventCallback( MQTTContext_t * pxMqttContext,
                              MQTTPacketInfo_t * pxPacketInfo,
                              MQTTDeserializedInfo_t * pxDeserializedInfo );

/**
 * @brief Process payload from /update/delta topic.
 *
 * This handler examines the version number and the powerOn state. If powerOn
 * state has changed, it sets a flag for the main function to take further actions.
 *
 * @param[in] pPublishInfo Deserialized publish info pointer for the incoming
 * packet.
 */
static void prvUpdateDeltaHandler( MQTTPublishInfo_t * pxPublishInfo );

/**
 * @brief Process payload from /update/accepted topic.
 *
 * This handler examines the accepted message that carries the same clientToken
 * as sent before.
 *
 * @param[in] pxPublishInfo Deserialized publish info pointer for the incoming
 * packet.
 */
static void prvUpdateAcceptedHandler( MQTTPublishInfo_t * pxPublishInfo );

/*function to publish reported state to device shadow*/
static BaseType_t PublishReportedState();

BaseType_t ReportedPubishStatus;

static char pcUpdateDocument[ SHADOW_REPORTED_JSON_LENGTH + 1 ] = { 0 };

static uint16_t usPublishPacketIdentifier;

#define mqttexampleTOPIC                                  democonfigCLIENT_IDENTIFIER "/device_shadow/test"

/* function to publish sample message to AWS IoT core on mqttexampleTOPIC */
static BaseType_t prvMQTTPublishToTopic( MQTTContext_t * pxMQTTContext );

/* function to initialize GPIO driver and turn led */
int LedInit();

/* Macro to GPIO PIN number of led_ds1 */
#define GPIO1_PIN14                     (14) /*< ds1 led connected to this gpio pin >*/

/* Macro to set GPIO port of led_ds1 */
#define GPIO1                           1

/*GPIO1 driver instance and pointer to GPIO1*/
extern  ARM_DRIVER_GPIO ARM_Driver_GPIO_(GPIO1);
ARM_DRIVER_GPIO *ptrDrv = &ARM_Driver_GPIO_(GPIO1);

/* GPIO related definitions */
uint8_t led_ds1 = GPIO1_PIN14;

/* Variable to get state of led, which is updated each time led state is changed. */
static uint32_t led_state=0;

/*-----------------------------------------------------------*/

static void prvUpdateDeltaHandler( MQTTPublishInfo_t * pxPublishInfo )
{
    static uint32_t ulCurrentVersion = 0; /* Remember the latestVersion # we've ever received */
    uint32_t ulVersion = 0U;
    uint32_t ulNewState = 0U;
    char * pcOutValue = NULL;
    uint32_t ulOutValueLength = 0U;
    JSONStatus_t result = JSONSuccess;

    assert( pxPublishInfo != NULL );
    assert( pxPublishInfo->pPayload != NULL );

    printf( "/update/delta json payload:%s.\n", ( const char * ) pxPublishInfo->pPayload );
    /* The payload will look similar to this:
     * {
     *      "version": 12,
     *      "timestamp": 1595437367,
     *      "state": {
     *          "powerOn": 1
     *      },
     *      "metadata": {
     *          "powerOn": {
     *          "timestamp": 1595437367
     *          }
     *      },
     *      "clientToken": "388062"
     *  }
     */

    /* Make sure the payload is a valid json document. */
    result = JSON_Validate( pxPublishInfo->pPayload,
                            pxPublishInfo->payloadLength );

    if( result == JSONSuccess )
    {
        /* Then we start to get the version value by JSON keyword "version". */
        result = JSON_Search( ( char * ) pxPublishInfo->pPayload,
                              pxPublishInfo->payloadLength,
                              "version",
                              sizeof( "version" ) - 1,
                              &pcOutValue,
                              ( size_t * ) &ulOutValueLength );
    }
    else
    {
        printf( "ERROR:The json document is invalid!!\n" );
    }

    if( result == JSONSuccess )
    {
        printf( "version: %.*s",ulOutValueLength,pcOutValue );
        /* Convert the extracted value to an unsigned integer value. */
        ulVersion = ( uint32_t ) strtoul( pcOutValue, NULL, 10 );
    }
    else
    {
        printf( "ERROR:No version in json document!!\n" );
    }

    printf( "version:%d, ulCurrentVersion:%d \r\n", ulVersion, ulCurrentVersion );
    /* When the version is much newer than the on we retained, that means the powerOn
     * state is valid for us. */
    if( ulVersion > ulCurrentVersion )
    {
        /* Set to received version as the current version. */
        ulCurrentVersion = ulVersion;

        /* Get powerOn state from json documents. */
        result = JSON_Search( ( char * ) pxPublishInfo->pPayload,
                              pxPublishInfo->payloadLength,
                              "state.powerOn",
                              sizeof( "state.powerOn" ) - 1,
                              &pcOutValue,
                              ( size_t * ) &ulOutValueLength );
    }
    else
    {
        /* In this demo, we discard the incoming message
         * if the version number is not newer than the latest
         * that we've received before. Your application may use a
         * different approach.
         */
        printf( "WARNING:The received version is smaller than current one!!\n" );
    }

    if( result == JSONSuccess )
    {
        /* Convert the powerOn state value to an unsigned integer value. */
        ulNewState = ( uint32_t ) strtoul( pcOutValue, NULL, 10 );

        printf( "The new power on state newState:%d, ulCurrentPowerOnState:%d \r\n",ulNewState, ulCurrentPowerOnState );

        if( (ulNewState == 0) || (ulNewState == 1) )
        {
            if( led_state == ulNewState )
            {
                printf( "******LED IS IN DESIRED STATE*****\n" );
            }
            else
            {
                if ( led_state == 1 )
                {
                    ptrDrv->SetValue(led_ds1, GPIO_PIN_OUTPUT_STATE_LOW);
                    led_state = 0;
                    printf( "----TOGGLING LED :: led state after change in delta : %d--------\n",led_state );
                }
                else
                {
                    ptrDrv->SetValue(led_ds1, GPIO_PIN_OUTPUT_STATE_HIGH);
                    led_state = 1;
                    printf( "----TOGGLING LED :: led state after change in delta : %d--------\n",led_state );
                }
            }
        }

        if( led_state != ulCurrentPowerOnState )
        {
            /* The received powerOn state is different from the one we retained before, so we switch them
             * and set the flag. */
            ulCurrentPowerOnState = ulNewState;
            stateChanged = true;
            ReportedPubishStatus =  PublishReportedState();
            while( ReportedPubishStatus != pdPASS );
            if( ReportedPubishStatus == pdPASS )
            {
                printf( "-----REPORTED state successfully published to AWS IoT Core device shadow-----\n" );
            }
        }
    }
    else
    {
        printf( "ERROR : No powerOn in json document!!\n" );
        xUpdateDeltaReturn = pdFAIL;
    }
}

/*-----------------------------------------------------------*/

static void prvUpdateAcceptedHandler( MQTTPublishInfo_t * pxPublishInfo )
{
    char * pcOutValue = NULL;
    uint32_t ulOutValueLength = 0U;
    uint32_t ulReceivedToken = 0U;
    JSONStatus_t result = JSONSuccess;

    assert( pxPublishInfo != NULL );
    assert( pxPublishInfo->pPayload != NULL );
    printf( "/update/accepted json payload:%s.\n", ( const char * ) pxPublishInfo->pPayload );
    /* Handle the reported state with state change in /update/accepted topic.
     * Thus we will retrieve the client token from the json document to see if
     * it's the same one we sent with reported state on the /update topic.
     * The payload will look similar to this:
     *  {
     *      "state": {
     *          "reported": {
     *          "powerOn": 1
     *          }
     *      },
     *      "metadata": {
     *          "reported": {
     *          "powerOn": {
     *              "timestamp": 1596573647
     *          }
     *          }
     *      },
     *      "version": 14698,
     *      "timestamp": 1596573647,
     *      "clientToken": "022485"
     *  }
     */

    /* Make sure the payload is a valid json document. */
    result = JSON_Validate( pxPublishInfo->pPayload,
                            pxPublishInfo->payloadLength );

    if( result == JSONSuccess )
    {
        /* Get clientToken from json documents. */
        result = JSON_Search( ( char * ) pxPublishInfo->pPayload,
                              pxPublishInfo->payloadLength,
                              "clientToken",
                              sizeof( "clientToken" ) - 1,
                              &pcOutValue,
                              ( size_t * ) &ulOutValueLength );
    }
    else
    {
        printf( "ERROR : Invalid json documents !!\n" );
    }

    if( result == JSONSuccess )
    {
        printf( "clientToken: %.*s", ulOutValueLength,pcOutValue );
        /* Convert the code to an unsigned integer value. */
        ulReceivedToken = ( uint32_t ) strtoul( pcOutValue, NULL, 10 );
        printf( "receivedToken:%d, clientToken:%u \r\n", ulReceivedToken, ulClientToken );
        /* If the clientToken in this update/accepted message matches the one we
         * published before, it means the device shadow has accepted our latest
         * reported state. We are done. */
        if( ulReceivedToken == ulClientToken )
        {    printf( "Received response from the device shadow. Previously published "
                    "update with clientToken=%u has been accepted.\n ", ulClientToken );
        }
        else
        {
             printf( "WARNING : The received clientToken=%u is not identical with the one=%u we sent\n",ulReceivedToken, ulClientToken );
        }
    }
    else
    {
         printf( "ERROR : No clientToken in json document!!\n" );
         xUpdateAcceptedReturn = pdFAIL;
    }
}


/*-----------------------------------------------------------*/

/* This is the callback function invoked by the MQTT stack when it receives
 * incoming messages. This function demonstrates how to use the Shadow_MatchTopic
 * function to determine whether the incoming message is a device shadow message
 * or not. If it is, it handles the message depending on the message type.
 */
static void prvEventCallback( MQTTContext_t * pxMqttContext,
                              MQTTPacketInfo_t * pxPacketInfo,
                              MQTTDeserializedInfo_t * pxDeserializedInfo )
{
    ShadowMessageType_t messageType = ShadowMessageTypeMaxNum;
    const char * pcThingName = NULL;
    uint16_t usThingNameLength = 0U;
    uint16_t usPacketIdentifier;

    ( void ) pxMqttContext;

    assert( pxDeserializedInfo != NULL );
    assert( pxMqttContext != NULL );
    assert( pxPacketInfo != NULL );

    usPacketIdentifier = pxDeserializedInfo->packetIdentifier;

    /* Handle incoming publish. The lower 4 bits of the publish packet
     * type is used for the dup, QoS, and retain flags. Hence masking
     * out the lower bits to check if the packet is publish. */
    if( ( pxPacketInfo->type & 0xF0U ) == MQTT_PACKET_TYPE_PUBLISH )
    {
        assert( pxDeserializedInfo->pPublishInfo != NULL );
        printf("pPublishInfo->pTopicName:%s.\n", pxDeserializedInfo->pPublishInfo->pTopicName);
        /* Let the Device Shadow library tell us whether this is a device shadow message. */
        if( SHADOW_SUCCESS == Shadow_MatchTopic( pxDeserializedInfo->pPublishInfo->pTopicName,
                                                 pxDeserializedInfo->pPublishInfo->topicNameLength,
                                                 &messageType,
                                                 &pcThingName,
                                                 &usThingNameLength ) )
        {
            /* Upon successful return, the messageType has been filled in. */
            if( messageType == ShadowMessageTypeUpdateDelta )
            {
                /* Handler function to process payload. */
                prvUpdateDeltaHandler( pxDeserializedInfo->pPublishInfo );
            }
            else if( messageType == ShadowMessageTypeUpdateAccepted )
            {
                /* Handler function to process payload. */
                prvUpdateAcceptedHandler( pxDeserializedInfo->pPublishInfo );
            }
            else if( messageType == ShadowMessageTypeUpdateDocuments )
            {
                printf( "/update/documents json payload:%s.\n", ( const char * ) pxDeserializedInfo->pPublishInfo->pPayload );
            }
            else if( messageType == ShadowMessageTypeUpdateRejected )
            {
                printf( "/update/rejected json payload:%s.\n", ( const char * ) pxDeserializedInfo->pPublishInfo->pPayload );
            }
            else
            {
                printf( "Other message type:%d !!\n", messageType );
            }
        }
        else
        {
            printf( "ERROR : Shadow_MatchTopic parse failed:%s !!\n",( const char * ) pxDeserializedInfo->pPublishInfo->pTopicName );
        }
    }
    else
    {
        vHandleOtherIncomingPacket( pxPacketInfo, usPacketIdentifier );
    }
}

/*-----------------------------------------------------------*/

/**
 * @brief Entry point of shadow demo.
 *
 * This main function demonstrates how to use the macros provided by the
 * Device Shadow library to assemble strings for the MQTT topics defined
 * by AWS IoT Device Shadow. It uses these macros for topics to subscribe
 * to:
 * - SHADOW_TOPIC_STRING_UPDATE_DELTA for "$aws/things/thingName/shadow/update/delta"
 * - SHADOW_TOPIC_STRING_UPDATE_ACCEPTED for "$aws/things/thingName/shadow/update/accepted"
 * - SHADOW_TOPIC_STRING_UPDATE_REJECTED for "$aws/things/thingName/shadow/update/rejected"
 *
 *It also uses these macros for topics to publish to:
 * -SHADOW_TOPIC_STRING_UPDATE for "$aws/things/thingName/shadow/update"
 * The helper functions this demo uses for MQTT operations have internal
 * loops to process incoming messages. Those are not the focus of this demo
 * and therefor, are placed in a separate file mqtt_demo_helpers.c.
 */
int RunDeviceShadowDemo( bool awsIotMqttMode,
                         const char * pIdentifier,
                         void * pNetworkServerInfo,
                         void * pNetworkCredentialInfo,
                         const void * pNetworkInterface )
{
    BaseType_t xDemoStatus = pdPASS;
    BaseType_t xDemoRunCount = 0UL;
    MQTTStatus_t status = MQTTSuccess;


    /* A buffer containing the update document. It has static duration to prevent
     * it from being placed on the call stack. */

    /* Remove compiler warnings about unused parameters. */
    ( void ) awsIotMqttMode;
    ( void ) pIdentifier;
    ( void ) pNetworkServerInfo;
    ( void ) pNetworkCredentialInfo;
    ( void ) pNetworkInterface;

    do
    {
        xDemoStatus = EstablishMqttSession( &xMqttContext,
                                            &xNetworkContext,
                                            &xBuffer,
                                            prvEventCallback );

        if( xDemoStatus == pdFAIL )
        {
            /* Log error to indicate connection failure. */
            printf( "ERROR : Failed to connect to MQTT broker.\n" );
        }
        else
        {
            /* Then try to subscribe Shadow delta and Shadow updated topics. */
            if( xDemoStatus == pdPASS )
            {
                xDemoStatus = SubscribeToTopic( &xMqttContext,
                                                SHADOW_TOPIC_STRING_UPDATE_DELTA( THING_NAME ),
                                                SHADOW_TOPIC_LENGTH_UPDATE_DELTA( THING_NAME_LENGTH ) );
            }

            if( xDemoStatus == pdPASS )
            {
                xDemoStatus = SubscribeToTopic( &xMqttContext,
                                                SHADOW_TOPIC_STRING_UPDATE_ACCEPTED( THING_NAME ),
                                                SHADOW_TOPIC_LENGTH_UPDATE_ACCEPTED( THING_NAME_LENGTH ) );
            }
            if( xDemoStatus == pdPASS )
            {
                xDemoStatus = SubscribeToTopic( &xMqttContext,
                                                SHADOW_TOPIC_STRING_UPDATE_REJECTED( THING_NAME ),
                                                SHADOW_TOPIC_LENGTH_UPDATE_REJECTED( THING_NAME_LENGTH ) );
            }
            if( xDemoStatus == pdPASS )
            {
                ( void ) memset( pcUpdateDocument, 0x00, sizeof( pcUpdateDocument ) );
//                snprintf( pcUpdateDocument,
//                          SHADOW_REPORTED_JSON_LENGTH + 1,
//                          SHADOW_REPORTED_JSON,
//                          ( int ) led_state,
//                          ( long unsigned ) ( xTaskGetTickCount() % 1000000 ) );
                ds1722_app();
                printf("---temperature : %f\n",temperature);
                snprintf( pcUpdateDocument,
                                          SHADOW_REPORTED_JSON_LENGTH + 1,
                                          SHADOW_REPORTED_JSON,
                                          ( float ) temperature,
										  (int)1,
										  (int)1,
                                          ( long unsigned ) ( xTaskGetTickCount() % 1000000 ) );
                xDemoStatus = PublishToTopic( &xMqttContext,
                                              SHADOW_TOPIC_STRING_UPDATE( THING_NAME ),
                                              SHADOW_TOPIC_LENGTH_UPDATE( THING_NAME_LENGTH ),
                                              pcUpdateDocument,
                                              ( SHADOW_REPORTED_JSON_LENGTH + 1 ) );
                //printf( "---published initial REPORTED led_state :: %d of device to AWS IoT Core device shadow----\n",led_state );
            }

        }
        while( 1 )
        {
            status = ProcessLoop( &xMqttContext, 5000 );
            xDemoStatus = prvMQTTPublishToTopic( &xMqttContext );
            if( xDemoStatus != pdPASS )
            {
                printf( "ERROR : Publishing Sample message failed\n" );
            }
        }
    } while( xDemoStatus != pdPASS );
    return( ( xDemoStatus == pdPASS ) ? EXIT_SUCCESS : EXIT_FAILURE );
}
/*Function to initilaze GPIO driver and turn on led */
int LedInit()
{
  /*
   * gpio1 pin14 is connected active high led ds1.
   */
    uint32_t ret = 0;

    ret = ptrDrv->Initialize(led_ds1, NULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf( "ERROR: Failed to initialize\n" );
        return ret;
    }
    ret = ptrDrv->PowerControl(led_ds1, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK)
    {
        printf("ERROR: Failed to powered full\n");
        goto error_uninitialize;
    }
    ret = ptrDrv->SetDirection(led_ds1, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK)
    {
        printf( "ERROR: Failed to configure\n" );
        goto error_power_off;
    }
    ret = ptrDrv->SetValue(led_ds1, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK )
    {
        printf( "ERROR: Failed to configure\n" );
        goto error_power_off;
    }
    led_state = 1;
    printf( "---Initial state of led set on device: %d----\n",led_state );
    return ret;

    error_power_off:
        ret = ptrDrv->PowerControl(led_ds1, ARM_POWER_OFF);
        if ((ret != ARM_DRIVER_OK) )
        {
            printf( "ERROR: Failed to power off \n" );
        }
        else
        {
            printf( "LEDs power off \n" );
        }

    error_uninitialize:
        ret = ptrDrv->Uninitialize (led_ds1);
        if ((ret != ARM_DRIVER_OK) )
        {
            printf("Failed to Un-initialize \n");
        }
        else
        {
            printf("Un-initialized \n");
        }
 }

/*function to publish reported state to device shadow */
static BaseType_t PublishReportedState()
{
    ( void ) memset( pcUpdateDocument, 0x00,sizeof( pcUpdateDocument ) );
    /* Keep the client token in global variable used to compare if
	                     * the same token in /update/accepted. */
    ulClientToken = ( xTaskGetTickCount() % 1000000 );
    snprintf( pcUpdateDocument,
              SHADOW_REPORTED_JSON_LENGTH + 1,
              SHADOW_REPORTED_JSON,
              ( int ) led_state,
              ( long unsigned ) ulClientToken );
    ReportedPubishStatus = PublishToTopic( &xMqttContext,
                                           SHADOW_TOPIC_STRING_UPDATE( THING_NAME ),
                                           SHADOW_TOPIC_LENGTH_UPDATE( THING_NAME_LENGTH ),
                                           pcUpdateDocument,
                                           ( SHADOW_REPORTED_JSON_LENGTH + 1 ) );
    return ReportedPubishStatus;
}

/*function to publish sample message to AWS IoT core on mqttexampleTOPIC*/
static BaseType_t prvMQTTPublishToTopic( MQTTContext_t * pxMQTTContext )
{
    MQTTStatus_t xResult;
    MQTTPublishInfo_t xMQTTPublishInfo;
    BaseType_t xStatus = pdPASS;
    char *str = "AlifSemiconductor";

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
