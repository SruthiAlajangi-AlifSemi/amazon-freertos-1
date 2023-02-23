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
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* Demo config. */
#include "defender_demo_config.h"

/* Metrics collector. */
#include "metrics_collector.h"

/* Report builder. */
#include "report_builder.h"

/* MQTT operations. */
#include "mqtt_demo_helpers.h"

/* JSON Library. */
#include "core_json.h"

/* Device Defender Client Library. */
#include "defender.h"

/**
 * @brief Predefined thing name.
 *
 * This is the example predefine thing name and could be compiled in ROM code.
 */
#define THING_NAME                        clientcredentialIOT_THING_NAME

/**
 * @brief The length of #THING_NAME.
 */
#define THING_NAME_LENGTH                 ( ( uint16_t ) ( sizeof( THING_NAME ) - 1 ) )

/**
 * @brief Number of seconds to wait for the response from AWS IoT Device
 * Defender service.
 */
#define DEFENDER_RESPONSE_WAIT_SECONDS    ( 2 )

/**
 * @brief The maximum number of times to run the loop in this demo.
 */
#ifndef DEFENDER_DEMO_MAX_ATTEMPTS
    #define DEFENDER_DEMO_MAX_ATTEMPTS    ( 3 )
#endif

/**
 * @brief Time in ticks to wait between each iteration of the demo execution,
 * in case a retry is required from demo execution failure.
 */
#define DELAY_BETWEEEN_DEMO_ATTEMPTS_TICKS    ( pdMS_TO_TICKS( 5000U ) )

/**
 * @brief Status values of the device defender report.
 */
typedef enum
{
    ReportStatusNotReceived,
    ReportStatusAccepted,
    ReportStatusRejected
} ReportStatus_t;

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
static MQTTContext_t mqttContext;

/**
 * @brief The network context used for OpenSSL operation.
 */
static NetworkContext_t networkContext;

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t sharedBuffer[ NETWORK_BUFFER_SIZE ];

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static MQTTFixedBuffer_t xBuffer =
{
    sharedBuffer,
    NETWORK_BUFFER_SIZE
};

/**
 * @brief Network Stats.
 */
static NetworkStats_t networkStats;

/**
 * @brief Open TCP ports array.
 */
static uint16_t openTcpPorts[ OPEN_TCP_PORTS_ARRAY_SIZE ];

/**
 * @brief Open UDP ports array.
 */
static uint16_t openUdpPorts[ OPEN_UDP_PORTS_ARRAY_SIZE ];

/**
 * @brief Established connections array.
 */
static Connection_t establishedConnections[ ESTABLISHED_CONNECTIONS_ARRAY_SIZE ];

/**
 * @brief All the metrics sent in the device defender report.
 */
static ReportMetrics_t deviceMetrics;

/**
 * @brief Report status.
 */
static ReportStatus_t reportStatus;

/**
 * @brief Buffer for generating the device defender report.
 */
static char deviceMetricsJsonReport[ DEVICE_METRICS_REPORT_BUFFER_SIZE ];

/**
 * @brief Report Id sent in the defender report.
 */
static uint32_t reportId = 0;

static uint16_t usPublishPacketIdentifier;

#define mqttexampleTOPIC                                  democonfigCLIENT_IDENTIFIER "/device_defender/test"

/*-----------------------------------------------------------*/

/* function to publish sample message to AWS IoT core on mqttexampleTOPIC */
static BaseType_t prvMQTTPublishToTopic( MQTTContext_t * pxMQTTContext );

/**
 * @brief Callback to receive the incoming publish messages from the MQTT broker.
 *
 * @param[in] pMqttContext MQTT context pointer.
 * @param[in] pPacketInfo Information on the type of incoming MQTT packet.
 * @param[in] pDeserializedInfo Deserialized information from incoming packet.
 */
static void publishCallback( MQTTContext_t * pMqttContext,
                             MQTTPacketInfo_t * pPacketInfo,
                             MQTTDeserializedInfo_t * pDeserializedInfo );

/**
 * @brief Collect all the metrics to be sent in the device defender report.
 *
 * On success, caller is responsible for freeing deviceMetrics.pTaskStatusArray.
 *
 * @return pdPASS if all the metrics are successfully collected;
 * pdFAIL otherwise.
 */
static BaseType_t collectDeviceMetrics( void );

/**
 * @brief Generate the device defender report.
 *
 * @param[out] pOutReportLength Length of the device defender report.
 *
 * @return pdPASS if the report is generated successfully;
 * pdFAIL otherwise.
 */
static BaseType_t generateDeviceMetricsReport( size_t * pOutReportLength );

/**
 * @brief Subscribe to the device defender topics.
 *
 * @param[in] pMqttContext MQTT context pointer.
 *
 * @return pdPASS if the subscribe is successful;
 * pdFAIL otherwise.
 */
static BaseType_t subscribeToDefenderTopics( MQTTContext_t * pMqttContext );

/**
 * @brief Unsubscribe from the device defender topics.
 *
 * @param[in] pMqttContext MQTT context pointer.
 *
 * @return pdPASS if the unsubscribe is successful;
 * pdFAIL otherwise.
 */
static BaseType_t unsubscribeFromDefenderTopics( MQTTContext_t * pMqttContext );

/**
 * @brief Validate the response received from the AWS IoT Device Defender Service.
 *
 * This functions checks that a valid JSON is received and the value of reportId
 * is same as was sent in the published report.
 *
 * @param[in] defenderResponse The defender response to validate.
 * @param[in] defenderResponseLength Length of the defender response.
 *
 * @return true if the response is valid;
 * false otherwise.
 */
static bool validateDefenderResponse( const char * defenderResponse,
                                      size_t defenderResponseLength );

int n=0;

/**
 * @brief MQTT packet type received from the MQTT broker.
 *
 * @note Only on receiving incoming PUBLISH, SUBACK, and UNSUBACK, this
 * variable is updated. For MQTT packets PUBACK and PINGRESP, the variable is
 * not updated since there is no need to specifically wait for it in this demo.
 * A single variable suffices as this demo uses single task and requests one operation
 * (of PUBLISH, SUBSCRIBE, UNSUBSCRIBE) at a time before expecting response from
 * the broker. Hence it is not possible to receive multiple packets of type PUBLISH,
 * SUBACK, and UNSUBACK in a single call of #prvWaitForPacket.
 * For a multi task application, consider a different method to wait for the packet, if needed.
 */
static uint16_t usPacketTypeReceived = 0U;


/**
 * @brief Process incoming Publish message.
 *
 * @param[in] pxPublishInfo is a pointer to structure containing deserialized
 * Publish message.
 */
static void prvMQTTProcessIncomingPublish( MQTTPublishInfo_t * pxPublishInfo );

/*-----------------------------------------------------------*/

static bool validateDefenderResponse( const char * defenderResponse,
                                      size_t defenderResponseLength )
{
    bool status = false;
    JSONStatus_t jsonResult = JSONSuccess;
    char * reportIdString = NULL;
    size_t reportIdStringLength;
    uint32_t reportIdInResponse;

    /* Is the response a valid JSON? */
    jsonResult = JSON_Validate( defenderResponse, defenderResponseLength );

    if( jsonResult != JSONSuccess )
    {
        LogError( ( "Invalid response from AWS IoT Device Defender Service: %.*s.",
                    ( int ) defenderResponseLength,
                    defenderResponse ) );
    }

    if( jsonResult == JSONSuccess )
    {
        /* Search for the reportId key in the response. */
        jsonResult = JSON_Search( ( char * ) defenderResponse,
                                  defenderResponseLength,
                                  "reportId",
                                  sizeof( "reportId" ) - 1,
                                  &( reportIdString ),
                                  &( reportIdStringLength ) );

        if( jsonResult != JSONSuccess )
        {
            LogError( ( "reportId key not found in the response from the"
                        "AWS IoT Device Defender Service: %.*s.",
                        ( int ) defenderResponseLength,
                        defenderResponse ) );
        }
    }

    if( jsonResult == JSONSuccess )
    {
        reportIdInResponse = ( uint32_t ) strtoul( reportIdString, NULL, 10 );

        /* Is the reportId present in the response same as was sent in the
         * published report? */
        if( reportIdInResponse == reportId )
        {
            LogInfo( ( "A valid response with reportId %lu received from the "
                       "AWS IoT Device Defender Service.",
                       ( unsigned long ) reportId ) );
            status = true;
        }
        else
        {
            LogError( ( "Unexpected reportId found in the response from the AWS"
                        "IoT Device Defender Service. Expected: %lu, Found: %lu, "
                        "Complete Response: %.*s.",
                        ( unsigned long ) reportIdInResponse,
                        ( unsigned long ) reportId,
                        ( int ) defenderResponseLength,
                        defenderResponse ) );
        }
    }

    return status;
}
/*-----------------------------------------------------------*/

static void publishCallback( MQTTContext_t * pMqttContext,
                             MQTTPacketInfo_t * pPacketInfo,
                             MQTTDeserializedInfo_t * pDeserializedInfo )
{
    DefenderStatus_t status;
    DefenderTopic_t api;
    bool validationResult;
    MQTTPublishInfo_t * pPublishInfo = pDeserializedInfo->pPublishInfo;

    /* Silence compiler warnings about unused variables. */
    ( void ) pMqttContext;

    /* Handle incoming publish. The lower 4 bits of the publish packet
     * type is used for the dup, QoS, and retain flags. Hence masking
     * out the lower bits to check if the packet is publish. */
    if( ( pPacketInfo->type & 0xF0U ) == MQTT_PACKET_TYPE_PUBLISH )
    {
        status = Defender_MatchTopic( pPublishInfo->pTopicName,
                                      pPublishInfo->topicNameLength,
                                      &( api ),
                                      NULL,
                                      NULL );

        if( status == DefenderSuccess )
        {
            if( api == DefenderJsonReportAccepted )
            {
                /* Check if the response is valid and is for the report we published. */
                validationResult = validateDefenderResponse( pPublishInfo->pPayload,
                                                             pPublishInfo->payloadLength );

                if( validationResult == true )
                {
                    printf("The defender report was accepted by the service. Response: %.*s.\n",
                               ( int ) pPublishInfo->payloadLength,
                               ( const char * ) pPublishInfo->pPayload);
                    reportStatus = ReportStatusAccepted;
                }
            }
            else if( api == DefenderJsonReportRejected )
            {
                /* Check if the response is valid and is for the report we published. */
                validationResult = validateDefenderResponse( pPublishInfo->pPayload,
                                                             pPublishInfo->payloadLength );

                if( validationResult == true )
                {
                    printf("The defender report was rejected by the service. Response: %.*s.\n",
                                ( int ) pPublishInfo->payloadLength,
                                ( const char * ) pPublishInfo->pPayload);
                    reportStatus = ReportStatusRejected;
                }
            }
            else
            {
                LogError( ( "Unexpected defender API : %d.", api ) );
            }
        }
        else
        {   prvMQTTProcessIncomingPublish( pDeserializedInfo->pPublishInfo );
            LogError( ( "Unexpected publish message received. Topic: %.*s, Payload: %.*s.",
                        ( int ) pPublishInfo->topicNameLength,
                        ( const char * ) pPublishInfo->pTopicName,
                        ( int ) pPublishInfo->payloadLength,
                        ( const char * ) ( pPublishInfo->pPayload ) ) );
        }
    }
    else
    {
        vHandleOtherIncomingPacket( pPacketInfo, pDeserializedInfo->packetIdentifier );
    }
}
/*-----------------------------------------------------------*/

static BaseType_t collectDeviceMetrics( void )
{
    BaseType_t status = pdFAIL;
    MetricsCollectorStatus_t metricsCollectorStatus;
    size_t numOpenTcpPorts, numOpenUdpPorts, numEstablishedConnections;
    UBaseType_t tasksWritten = 0U;
    TaskStatus_t taskStatus = { 0 };
    TaskStatus_t * pTaskStatusArray = NULL;
    UBaseType_t numTasksRunning;

    /* Collect bytes and packets sent and received. */
    metricsCollectorStatus = GetNetworkStats( &( networkStats ) );

    if( metricsCollectorStatus != MetricsCollectorSuccess )
    {
        LogError( ( "GetNetworkStats failed. Status: %d.",
                    metricsCollectorStatus ) );
    }

    /* Collect a list of open TCP ports. */
    if( metricsCollectorStatus == MetricsCollectorSuccess )
    {
        metricsCollectorStatus = GetOpenTcpPorts( &( openTcpPorts[ 0 ] ),
                                                  OPEN_TCP_PORTS_ARRAY_SIZE,
                                                  &( numOpenTcpPorts ) );

        if( metricsCollectorStatus != MetricsCollectorSuccess )
        {
            LogError( ( "GetOpenTcpPorts failed. Status: %d.",
                        metricsCollectorStatus ) );
        }
    }

    /* Collect a list of open UDP ports. */
    if( metricsCollectorStatus == MetricsCollectorSuccess )
    {
        metricsCollectorStatus = GetOpenUdpPorts( &( openUdpPorts[ 0 ] ),
                                                  OPEN_UDP_PORTS_ARRAY_SIZE,
                                                  &( numOpenUdpPorts ) );

        if( metricsCollectorStatus != MetricsCollectorSuccess )
        {
            LogError( ( "GetOpenUdpPorts failed. Status: %d.",
                        metricsCollectorStatus ) );
        }
    }

    /* Collect a list of established connections. */
    if( metricsCollectorStatus == MetricsCollectorSuccess )
    {
        metricsCollectorStatus = GetEstablishedConnections( &( establishedConnections[ 0 ] ),
                                                            ESTABLISHED_CONNECTIONS_ARRAY_SIZE,
                                                            &( numEstablishedConnections ) );

        if( metricsCollectorStatus != MetricsCollectorSuccess )
        {
            LogError( ( "GetEstablishedConnections failed. Status: %d.",
                        metricsCollectorStatus ) );
        }
    }

    if( metricsCollectorStatus == MetricsCollectorSuccess )
    {
        /* Get task count */
        numTasksRunning = uxTaskGetNumberOfTasks();

        /* Allocate pTaskStatusArray */
        pTaskStatusArray = pvPortMalloc( numTasksRunning * sizeof( TaskStatus_t ) );

        if( pTaskStatusArray == NULL )
        {
            LogError( ( "Cannot allocate memory for pTaskStatusArray array: pvPortMalloc() failed." ) );
            metricsCollectorStatus = MetricsCollectorCollectionFailed;
        }
    }

    /* Collect custom metrics from the system to send to AWS IoT Device Defender.
     * This demo sends this task's stack high water mark as a "number" type
     * custom metric and the current task ids as a "list of numbers" type custom
     * metric. */
    if( metricsCollectorStatus == MetricsCollectorSuccess )
    {
        /* Get the current task's status information. The usStackHighWaterMark
         * field of the task status will be included in the report as a "number"
         * custom metric. */
        vTaskGetInfo(
            /* NULL has the function query this task. */
            NULL,
            &taskStatus,
            /* Include the stack high water mark value. */
            pdTRUE,
            /* Don't include the task state in the TaskStatus_t structure. */
            eRunning );

        /* Get the task status information for all running tasks. The task IDs
         * of each task is then extracted to include in the report as a "list of
         * numbers" custom metric */
        tasksWritten = uxTaskGetSystemState( pTaskStatusArray, numTasksRunning, NULL );

        if( tasksWritten == 0 )
        {
            /* If 0 is returned, the buffer was too small. This line is reached
             * when we hit the race condition where tasks have been added since
             * we got the result of uxTaskGetNumberOfTasks() */
            metricsCollectorStatus = MetricsCollectorCollectionFailed;
            LogError( ( "Failed to collect task IDs. uxTaskGetSystemState() failed due to insufficient buffer space." ) );
        }
    }

    /* Populate device metrics. */
    if( metricsCollectorStatus == MetricsCollectorSuccess )
    {
        status = pdPASS;
        deviceMetrics.pNetworkStats = &( networkStats );
        deviceMetrics.pOpenTcpPortsArray = &( openTcpPorts[ 0 ] );
        deviceMetrics.openTcpPortsArrayLength = numOpenTcpPorts;
        deviceMetrics.pOpenUdpPortsArray = &( openUdpPorts[ 0 ] );
        deviceMetrics.openUdpPortsArrayLength = numOpenUdpPorts;
        deviceMetrics.pEstablishedConnectionsArray = &( establishedConnections[ 0 ] );
        deviceMetrics.establishedConnectionsArrayLength = numEstablishedConnections;
        deviceMetrics.stackHighWaterMark = taskStatus.usStackHighWaterMark;
        deviceMetrics.pTaskStatusArray = pTaskStatusArray;
        deviceMetrics.taskStatusArrayLength = tasksWritten;
    }
    else
    {
        /* Free pTaskStatusArray if we allocated it but did not add it to the
         * deviceMetrics stuct. */
        if( pTaskStatusArray != NULL )
        {
            vPortFree( pTaskStatusArray );
        }
    }

    return status;
}
/*-----------------------------------------------------------*/

static BaseType_t subscribeToDefenderTopics( MQTTContext_t * pMqttContext )
{
    BaseType_t status = pdFAIL;

    status = SubscribeToTopic( pMqttContext,
                               DEFENDER_API_JSON_ACCEPTED( THING_NAME ),
                               DEFENDER_API_LENGTH_JSON_ACCEPTED( THING_NAME_LENGTH ) );

    if( status == pdPASS )
    {
        status = SubscribeToTopic( pMqttContext,
                                   DEFENDER_API_JSON_REJECTED( THING_NAME ),
                                   DEFENDER_API_LENGTH_JSON_REJECTED( THING_NAME_LENGTH ) );
    }

    return status;
}
/*-----------------------------------------------------------*/

static BaseType_t unsubscribeFromDefenderTopics( MQTTContext_t * pMqttContext )
{
    BaseType_t status = pdFAIL;

    status = UnsubscribeFromTopic( pMqttContext,
                                   DEFENDER_API_JSON_ACCEPTED( THING_NAME ),
                                   DEFENDER_API_LENGTH_JSON_ACCEPTED( THING_NAME_LENGTH ) );

    if( status == pdPASS )
    {
        status = UnsubscribeFromTopic( pMqttContext,
                                       DEFENDER_API_JSON_REJECTED( THING_NAME ),
                                       DEFENDER_API_LENGTH_JSON_REJECTED( THING_NAME_LENGTH ) );
    }

    return status;
}
/*-----------------------------------------------------------*/

static BaseType_t generateDeviceMetricsReport( size_t * pOutReportLength )
{
    BaseType_t status = pdFAIL;
    ReportBuilderStatus_t reportBuilderStatus;

    /* Generate the metrics report in the format expected by the AWS IoT Device
     * Defender Service. */
    reportBuilderStatus = GenerateJsonReport( &( deviceMetricsJsonReport[ 0 ] ),
                                              DEVICE_METRICS_REPORT_BUFFER_SIZE,
                                              &( deviceMetrics ),
                                              DEVICE_METRICS_REPORT_MAJOR_VERSION,
                                              DEVICE_METRICS_REPORT_MINOR_VERSION,
                                              reportId,
                                              pOutReportLength );
    printf("Metrics report :%s\n",deviceMetricsJsonReport);
    if( reportBuilderStatus != ReportBuilderSuccess )
    {
        LogError( ( "GenerateJsonReport failed. Status: %d.",
                    reportBuilderStatus ) );
    }
    else
    {
        LogDebug( ( "Generated Report: %.*s.",
                    ( int ) ( *pOutReportLength ),
                    &( deviceMetricsJsonReport[ 0 ] ) ) );
        status = pdPASS;
    }

    return status;
}
/*-----------------------------------------------------------*/

/**
 * @brief The function that runs the Defender demo, called by the demo runner.
 *
 * @param[in] awsIotMqttMode Ignored for the Defender demo.
 * @param[in] pIdentifier Ignored for the Defender demo.
 * @param[in] pNetworkServerInfo Ignored for the Defender demo.
 * @param[in] pNetworkCredentialInfo Ignored for the Defender demo.
 * @param[in] pNetworkInterface Ignored for the Defender demo.
 *
 * @return `EXIT_SUCCESS` if the demo completes successfully; `EXIT_FAILURE` otherwise.
 */

int RunDeviceDefenderDemo( bool awsIotMqttMode,
                           const char * pIdentifier,
                           void * pNetworkServerInfo,
                           void * pNetworkCredentialInfo,
                           const void * pNetworkInterface )
{
    BaseType_t demoStatus = pdFAIL;
    size_t reportLength = 0U, i;
    bool mqttSessionEstablished = false;
    UBaseType_t demoRunCount = 0;
    BaseType_t retryDemoLoop = pdFALSE;

    ( void ) awsIotMqttMode;
    ( void ) pIdentifier;
    ( void ) pNetworkServerInfo;
    ( void ) pNetworkCredentialInfo;
    ( void ) pNetworkInterface;

    /* This demo runs a single loop unless there are failures in the demo execution.
     * In case of failures in the demo execution, demo loop will be retried for up to
     * DEFENDER_DEMO_MAX_ATTEMPTS times. */
    do
    {
        /* Start with report not received. */
        reportStatus = ReportStatusNotReceived;

        /* Set a report Id to be used.
         *
         * !!!NOTE!!!
         * This demo sets the report ID to xTaskGetTickCount(), which may collide
         * if the device is reset. Reports for a Thing with a previously used
         * report ID will be assumed to be duplicates and discarded by the Device
         * Defender service. The report ID needs to be unique per report sent with
         * a given Thing. We recommend using an increasing unique id such as the
         * current timestamp. */
        reportId = ( uint32_t ) xTaskGetTickCount();

        LogInfo( ( "Establishing MQTT session..." ) );
        demoStatus = EstablishMqttSession( &mqttContext,
                                           &networkContext,
                                           &xBuffer,
                                           publishCallback );

        if( demoStatus == pdFAIL )
        {
            printf("Failed to establish MQTT session.\n");

        }
        else
        {
            mqttSessionEstablished = true;
        }

        if( demoStatus == pdPASS )
        {
            printf("Subscribing to defender topics...\n");
            demoStatus = subscribeToDefenderTopics( &mqttContext );

            if( demoStatus == pdFAIL )
            {
                printf("Failed to subscribe to defender topics.\n");
            }
        }
        if( demoStatus == pdPASS )
        {
            demoStatus = SubscribeToTopic( &mqttContext,
                                           mqttexampleTOPIC,
                                           strlen ( mqttexampleTOPIC ) );
        }


        if( demoStatus == pdPASS )
        {
            printf("Collecting device metrics...\n");
            demoStatus = collectDeviceMetrics();

            if( demoStatus == pdFAIL )
            {
                printf("Failed to collect device metrics.\n");
            }
        }

        if( demoStatus == pdPASS )
        {
            printf("Generating device defender report...\n");
            demoStatus = generateDeviceMetricsReport( &( reportLength ) );

            /* Free the allocated array in deviceMetrics struct which is not
             * used anymore after generateDeviceMetricsReport(). This code is
             * only reached when collectDeviceMetrics succeeded, so
             * deviceMetrics.pTaskStatusArray is a valid allocation that needs
             * to be freed. */
            vPortFree( deviceMetrics.pTaskStatusArray );

            if( demoStatus == pdFAIL )
            {
                printf("Failed to generate device defender report.\n");
            }
        }

        if( demoStatus == pdPASS )
        {
            printf("Publishing device defender report...\n");
            demoStatus = PublishToTopic( &mqttContext,
                                         DEFENDER_API_JSON_PUBLISH( THING_NAME ),
                                         DEFENDER_API_LENGTH_JSON_PUBLISH( THING_NAME_LENGTH ),
                                         &( deviceMetricsJsonReport[ 0 ] ),
                                         reportLength );

            if( demoStatus == pdFAIL )
            {
                printf("Failed to publish device defender report.\n");
            }
        }

        if( demoStatus == pdPASS )
        {
            /* Note that PublishToTopic already called MQTT_ProcessLoop, therefore
             * responses may have been received and the publishCallback may have
             * been called. */
            for( i = 0; i < DEFENDER_RESPONSE_WAIT_SECONDS; i++ )
            {
                /* reportStatus is updated in the publishCallback. */
                if( reportStatus != ReportStatusNotReceived )
                {
                    break;
                }

                ( void ) ProcessLoop( &mqttContext, 1000 );
            }
        }

        if( reportStatus == ReportStatusNotReceived )
        {
            printf("Failed to receive response from AWS IoT Device Defender Service.\n");
            demoStatus = pdFAIL;
        }
        if( demoStatus == pdPASS )
        {
            for(int i=0;i<14;i++)
            {
                demoStatus = prvMQTTPublishToTopic( &mqttContext );
                if(demoStatus != pdPASS)
                {
                   printf(" %dth message failed to publish to AWS IoT Core\n",i);
                }
            }
        }


        /* Unsubscribe and disconnect if MQTT session was established. Per the MQTT
         * protocol spec, it is okay to send UNSUBSCRIBE even if no corresponding
         * subscription exists on the broker. Therefore, it is okay to attempt
         * unsubscribe even if one more subscribe failed earlier. */
        if( mqttSessionEstablished )
        {
            printf("Unsubscribing from defender topics...\n");
            demoStatus = unsubscribeFromDefenderTopics( &mqttContext );

            if( demoStatus == pdFAIL )
            {
                printf("Failed to unsubscribe from defender topics.\n");
            }
            printf("Closing MQTT session...\n");
            ( void ) DisconnectMqttSession( &mqttContext, &networkContext );
        }

        /* Increment the demo run count. */
        demoRunCount++;

        if( ( demoStatus == pdPASS ) && ( reportStatus == ReportStatusAccepted ) )
        {
            printf("Demo completed successfully.\n");
            /* Reset the flag for demo retry. */
            retryDemoLoop = pdFALSE;
        }
        else
        {
            demoStatus = pdFAIL;

            if( demoRunCount < DEFENDER_DEMO_MAX_ATTEMPTS )
            {
                printf("Demo iteration %lu failed. Retrying...\n",
                           ( unsigned long ) demoRunCount );
                retryDemoLoop = pdTRUE;


                /* Clear the flag indicating successful MQTT session establishment
                 * before attempting a retry. */
                mqttSessionEstablished = false;

                LogInfo( ( "A short delay before the next demo iteration." ) );
                vTaskDelay( DELAY_BETWEEEN_DEMO_ATTEMPTS_TICKS );
            }
            else
            {
                printf("All %lu demo iterations failed.\n",
                            ( unsigned long ) DEFENDER_DEMO_MAX_ATTEMPTS );
                retryDemoLoop = pdFALSE;
            }
        }
    } while( retryDemoLoop == pdTRUE );

    return( ( demoStatus == pdPASS ) ? EXIT_SUCCESS : EXIT_FAILURE );
}

/*function to publish sample message to AWS IoT core on mqttexampleTOPIC*/
static BaseType_t prvMQTTPublishToTopic( MQTTContext_t * pxMQTTContext )
{
    MQTTStatus_t xResult;
    MQTTPublishInfo_t xMQTTPublishInfo;
    BaseType_t xStatus = pdPASS;
    n = n+1;
    char *str = "AlifSemiconductor";
    char str2[30];
    sprintf(str2,"message %d :",n);
    strcat(str2,str);

    /* Some fields are not used by this demo so start with everything at 0. */
    ( void ) memset( ( void * ) &xMQTTPublishInfo, 0x00, sizeof( xMQTTPublishInfo ) );

    /* This demo uses QoS1. */
    xMQTTPublishInfo.qos = MQTTQoS1;
    xMQTTPublishInfo.retain = false;
    xMQTTPublishInfo.pTopicName = mqttexampleTOPIC;
    xMQTTPublishInfo.topicNameLength = ( uint16_t ) strlen( mqttexampleTOPIC );
    xMQTTPublishInfo.pPayload = str2;
    xMQTTPublishInfo.payloadLength = strlen(str2);

    /* Get a unique packet id. */
    usPublishPacketIdentifier = MQTT_GetPacketId( pxMQTTContext );

    /* Send PUBLISH packet. Packet ID is not used for a QoS1 publish. */
    xResult = MQTT_Publish( pxMQTTContext, &xMQTTPublishInfo, usPublishPacketIdentifier );

    if( xResult != MQTTSuccess )
    {
        xStatus = pdFAIL;
        printf( "Failed to send PUBLISH message to broker: Topic=%s, Error=%s",
                mqttexampleTOPIC,
                MQTT_Status_strerror( xResult ) );
    }

    return xStatus;
}

static void prvMQTTProcessIncomingPublish( MQTTPublishInfo_t * pxPublishInfo )
{
    configASSERT( pxPublishInfo != NULL );

    /* Set the global for indicating that an incoming publish is received. */
    usPacketTypeReceived = MQTT_PACKET_TYPE_PUBLISH;
    static int counter =0;
    counter++;
    /* Process incoming Publish. */
    LogInfo( ( "Incoming QoS : %d\n", pxPublishInfo->qos ) );
    /* Verify the received publish is for the we have subscribed to. */
    if( ( pxPublishInfo->topicNameLength == strlen( mqttexampleTOPIC ) ) &&
        ( 0 == strncmp( mqttexampleTOPIC, pxPublishInfo->pTopicName, pxPublishInfo->topicNameLength ) ) )
    {
        printf( "counter : %d\n",counter );
        printf( "Incoming Publish Topic Name: %.*s matches subscribed topic. Incoming Publish Message : %.*s\n",
                 pxPublishInfo->topicNameLength,pxPublishInfo->pTopicName,pxPublishInfo->payloadLength,pxPublishInfo->pPayload );

    }
    else
    {
        printf( "Incoming Publish Topic Name: %.*s does not match subscribed topic.\n",
                   pxPublishInfo->topicNameLength,
                   pxPublishInfo->pTopicName );
    }
}
/*-----------------------------------------------------------*/
