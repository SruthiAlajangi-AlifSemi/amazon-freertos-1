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
 * @file     NetworkInterface.c
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     12-Feb-2021
 * @brief    Implements the Ethernet MAC NetworkInterface Driver.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_UDP_IP.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkInterface.h"
#include "NetworkBufferManagement.h"

#include "phyHandling.h"

/* Driver includes */
#include "eqos.h"
#include "Driver_PINMUX_AND_PINPAD.h"

/* Generic includes */
#if ( ipconfigHAS_DEBUG_PRINTF != 0 ) || ( ipconfigHAS_PRINTF != 0 )
    #include <stdio.h>
#endif

#define HANDLER_TASK_STACK_SIZE         140ul /**< Stack size of the FreeRTOS task that handles Rx frames. */

#ifndef niDESCRIPTOR_WAIT_TIME_MS
	#define niDESCRIPTOR_WAIT_TIME_MS   250ul /**< Maximum time to wait for a free Tx Descriptor. */
#endif

/* private functions */

/* Initialise the hardware */
static BaseType_t prvEqosHwInit( DwEqosDev * pxEthDev );

/* Set up descriptors */
static void prvInitDescriptors( DwEqosDev * pxEthDev );
static void prvInitTxDescs( DwEqosDev * pxEthDev );
static void prvInitRxDescs( DwEqosDev * pxEthDev );

/* Task to process received Frames */
static void prvEqosHandlerTask( void * pvParamaters );

/* set a given mac address in the hardware */
static void prvSetMacAddr( DwEqosDev * pxEthDev, const uint8_t * ucAddress );

/* MAC instance used */
static DwEqosDev EQOS0 = {
    .pxRegs = (volatile DW_EQOS_REGS *) ETH_BASE,
    .xIrq = ETH_SBD_IRQ,
#if ( ipconfigETHERNET_AN_ENABLE != 0 )
    .Config = {
        .ucAutoNegCtrl = AN_ENABLE,
    },
#else
	.Config = {
	        .ucAutoNegCtrl = AN_DISABLE,
	#if ( ipconfigETHERNET_USE_100MB != 0 )
			.ucSpeed  = ETH_SPEED_100M,
    #else
			.ucSpeed  = ETH_SPEED_10M,
    #endif

    #if ( ipconfigETHERNET_USE_FULL_DUPLEX != 0 )
			.ucDuplex = ETH_DUPLEX_FULL,
	#else
			.ucDuplex = ETH_DUPLEX_HALF,
    #endif
	},
#endif
};

/** @brief Flag receive buffer unavailable condition. */
static volatile uint32_t ulRbuFlag;

/** @brief Our MAC address, defined in the application. */
extern uint8_t ucMACAddress[ 6 ];

/*
 * Events signalled from the ISR. Note that Receive buffer unavailable
 * condition is currently indicated as an Rx event.
 */
#define EQOS_RX_EVENT     1UL /**< Signal a Rx event to the task. */
#define EQOS_TX_EVENT     2UL /**< Signal a Tx complete event to the task. */
#define EQOS_ALL_EVENT    ( EQOS_RX_EVENT | EQOS_TX_EVENT )

/** @brief Pending ISR Events to be processed */
static volatile uint32_t  ulISREvents;

/** @brief FreeRTOS phyhandling PHY object. */
static EthernetPhy_t xPhyObject;

/** @brief FreeRTOS PHY properties object. */
const PhyProperties_t xPHYProperties =
{
#if ( ipconfigETHERNET_AN_ENABLE != 0 )
    .ucSpeed      = PHY_SPEED_AUTO,
    .ucDuplex     = PHY_DUPLEX_AUTO,
#else
    #if ( ipconfigETHERNET_USE_100MB != 0 )
        .ucSpeed  = PHY_SPEED_100,
    #else
        .ucSpeed  = PHY_SPEED_10,
    #endif

    #if ( ipconfigETHERNET_USE_FULL_DUPLEX != 0 )
        .ucDuplex = PHY_DUPLEX_FULL,
    #else
        .ucDuplex = PHY_DUPLEX_HALF,
    #endif
#endif /* if ( ipconfigETHERNET_AN_ENABLE != 0 ) */

#if ( ipconfigETHERNET_AN_ENABLE != 0 ) && ( ipconfigETHERNET_AUTO_CROSS_ENABLE != 0 )
    .ucMDI_X      = PHY_MDIX_AUTO,
#elif ( ipconfigETHERNET_CROSSED_LINK != 0 )
    .ucMDI_X      = PHY_MDIX_CROSSED,
#else
    .ucMDI_X      = PHY_MDIX_DIRECT,
#endif
};

/** @brief Handle for the task that handles Rx Packets. */
static TaskHandle_t xRxHandlerTask = NULL;

/** @brief Counting semaphore for resource handling of Tx descriptors. */
static SemaphoreHandle_t xTXDescriptorSemaphore = NULL;

/** @brief Rx DMA Descriptors */
static DmaDesc RxDmaDescs[ RX_DESC_COUNT ]__attribute__( ( section( "eth_buf" ) ) ) __attribute__( ( aligned( 16 ) ) );

/** @brief Tx DMA Descriptors */
static DmaDesc TxDmaDescs[ TX_DESC_COUNT ]__attribute__( ( section( "eth_buf" ) ) ) __attribute__( ( aligned( 16 ) ) );

/** @brief Rx Buffer Descriptors */
static NetworkBufferDescriptor_t *RxBufferDescs[ RX_DESC_COUNT ];

#if ( ipconfigZERO_COPY_RX_DRIVER == 0 )
/** @brief Static Rx buffers, used only if zero copy rx is disabled. */
static uint32_t RxBuffers[ RX_DESC_COUNT ][ ETH_BUF_SIZE >> 2 ]__attribute__( ( section( "eth_buf" ) ) );
#endif /* ipconfigZERO_COPY_RX_DRIVER */

#if ( ipconfigZERO_COPY_TX_DRIVER == 0 )
/** @brief Static tx buffers, used only if zero copy tx is disabled. */
static uint32_t TxBuffers[ TX_DESC_COUNT ][ ETH_BUF_SIZE >> 2 ]__attribute__( ( section("eth_buf") ) );
#endif /* ipconfigZERO_COPY_TX_DRIVER */

/**
 * @brief Initialize the Tx DMA Descriptors
 *
 * @param[in]: pxEthDev Pointer to the DwEqosDev structure.
 */
static void prvInitTxDescs( DwEqosDev * pxEthDev )
{
    uint32_t ul;

    for ( ul = 0; ul < TX_DESC_COUNT; ul++ )
        pxEthDev->TxDescs[ ul ] = ( DmaDesc ) { 0, 0, 0, 0 };

    pxEthDev->pxRegs->DMA_CH0_TX_BASE_ADDR = ( uint32_t ) LocalToGlobal( pxEthDev->TxDescs );
    pxEthDev->pxRegs->DMA_CHO_TX_RING_LEN = TX_DESC_COUNT - 1;

    SCB_CleanDCache_by_Addr( ( uint32_t * ) pxEthDev->TxDescs, TX_DESC_COUNT * sizeof( DmaDesc ) );
}

/**
 * @brief Initialize the Rx DMA Descriptors
 *
 * @param[in]: pxEthDev Pointer to the DwEqosDev structure.
 */
static void prvInitRxDescs( DwEqosDev * pxEthDev )
{
    uint32_t ul, ulLastRxDesc;
    DmaDesc * pxDesc;

    for (ul = 0; ul < RX_DESC_COUNT; ul++)
    {
        pxDesc = &pxEthDev->RxDescs[ ul ];
        #if ( ipconfigZERO_COPY_RX_DRIVER != 0 )
        {
            NetworkBufferDescriptor_t * pxBuffer;
            pxBuffer = pxGetNetworkBufferWithDescriptor( ETH_BUF_SIZE, 100ul );

            configASSERT( pxBuffer != NULL );

            if( pxBuffer != NULL )
            {
                pxDesc->ulDes0 = ( uint32_t ) ( LocalToGlobal( pxBuffer->pucEthernetBuffer ) );
                /* save the NetworkBufferDescriptor_t pointer for this DMA descriptor */
                pxEthDev->pxRxBufferDescs[ ul ] = pxBuffer;
            }
        }
        #else /* if ( ipconfigZERO_COPY_RX_DRIVER != 0 ) */
        {
            pxDesc->ulDes0 = ( uint32_t ) ( LocalToGlobal( &RxBuffers[ ul ][ 0 ] ) );
        }
        #endif
        pxDesc->ulDes3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN | RDES3_BUFFER1_VALID_ADDR;
    }


    pxEthDev->pxRegs->DMA_CH0_RX_BASE_ADDR = ( uint32_t ) LocalToGlobal( pxEthDev->RxDescs );
    pxEthDev->pxRegs->DMA_CH0_RX_RING_LEN = RX_DESC_COUNT - 1;

    ulLastRxDesc = ( uint32_t ) &( pxEthDev->RxDescs[ RX_DESC_COUNT - 1 ] );

    pxEthDev->pxRegs->DMA_CH0_RX_END_ADDR = ulLastRxDesc;

    SCB_CleanDCache_by_Addr( ( uint32_t * ) pxEthDev->RxDescs, RX_DESC_COUNT * sizeof( DmaDesc ) );
}

/**
 * @brief Set the chosen MAC address in the controller's register.
 *
 * @param[in] pxEthDev: Pointer to the DwEqosDev structure.
 * @param[in] ucAddress: MAC Address
 */
static void prvSetMacAddr( DwEqosDev * pxEthDev, const uint8_t * ucAddress )
{
    uint32_t ulHi, ulLo;

    if ( !ucAddress )
        return;

    ulHi = ( ucAddress[ 5 ] <<  8) |  ucAddress[ 4 ] | EQOS_MAC_ADDR_HIGH_AE;
    ulLo = ( ucAddress[ 3 ] << 24) | (ucAddress[ 2 ] << 16) |
            ( ucAddress[ 1 ] <<  8) |  ucAddress[ 0 ];

    pxEthDev->pxRegs->MAC_ADDR_LOW_0 = ulLo;
    pxEthDev->pxRegs->MAC_ADDR_HIGH_0 = ulHi;
}

/**
 * @brief MAC Hardware instance specific utility function for Eqos_PhyRead().
 *
 * @param[in] xAddress: The PHY address.
 * @param[in] xRegister: Register address.
 * @param[out] pulData: Address to store the read value.
 * @param[in] pxEthDev: Pointer to the DwEqosDev structure.
 */
BaseType_t EqosHwPhyRead( BaseType_t xAddress, BaseType_t xRegister, uint32_t * pulData,
        DwEqosDev * pxEthDev )
{
    uint32_t ulVal, ulTimeout = 5;

    ulVal = ( xAddress << EQOS_MAC_MDIO_ADDR_PA_SHIFT ) |
            ( xRegister << EQOS_MAC_MDIO_ADDR_RDA_SHIFT ) |
            ( EQOS_MAC_MDIO_ADDR_GOC_READ << EQOS_MAC_MDIO_ADDR_GOC_SHIFT ) |
            ( EQOS_MAC_MDIO_ADDR_CR_150_250 << EQOS_MAC_MDIO_ADDR_CR_SHIFT )|
            EQOS_MAC_MDIO_ADDR_GB;

    pxEthDev->pxRegs->MAC_MDIO_ADDR = ulVal;

    do
    {
        if ( ! ( pxEthDev->pxRegs->MAC_MDIO_ADDR & EQOS_MAC_MDIO_ADDR_GB ) )
        {
            *pulData = pxEthDev->pxRegs->MAC_MDIO_DATA;
            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 1 ) );
        ulTimeout--;
    } while ( ulTimeout );

    if ( !ulTimeout )
        return pdFAIL;

    return pdPASS;
}

/**
 * @brief MAC Hardware instance specific utility function for Eqos_PhyWrite()
 *
 * @param[in] xAddress: The PHY address.
 * @param[in] xRegister: Register address.
 * @param[in] ulData: Value to be written.
 * @param[in] pxEthDev: Pointer to the DwEqosDev structure.
 */
BaseType_t EqosHwPhyWrite( BaseType_t xAddress, BaseType_t xRegister, uint32_t ulData,
    DwEqosDev * pxEthDev)
{
    uint32_t ulVal, ulTimeout = 5;

    ulVal = ( xAddress << EQOS_MAC_MDIO_ADDR_PA_SHIFT ) |
                ( xRegister << EQOS_MAC_MDIO_ADDR_RDA_SHIFT ) |
                ( EQOS_MAC_MDIO_ADDR_GOC_WRITE << EQOS_MAC_MDIO_ADDR_GOC_SHIFT ) |
                ( EQOS_MAC_MDIO_ADDR_CR_150_250 << EQOS_MAC_MDIO_ADDR_CR_SHIFT ) |
                EQOS_MAC_MDIO_ADDR_GB;

    pxEthDev->pxRegs->MAC_MDIO_DATA = ulData;
    pxEthDev->pxRegs->MAC_MDIO_ADDR = ulVal;

    do {
        if( !( pxEthDev->pxRegs->MAC_MDIO_ADDR & EQOS_MAC_MDIO_ADDR_GB ) )
            break;
        vTaskDelay( pdMS_TO_TICKS( 1 ) );
        ulTimeout--;
    } while( ulTimeout );

    if ( !ulTimeout )
        return pdFAIL;

    return pdPASS;
}

/**
 * @brief Read a PHY register over MDIO.
 *
 * @param[in] xAddress: The PHY address.
 * @param[in] xRegister: Register address.
 * @param[out] pulValue: Address to store the read value.
 */
BaseType_t Eqos_PhyRead( BaseType_t xAddress, BaseType_t xRegister, uint32_t * pulValue )
{
    return EqosHwPhyRead( xAddress, xRegister, pulValue, &EQOS0 );
}

/**
 * @brief Write into a PHY register over MDIO.
 *
 * @param[in] xAddress: The PHY address.
 * @param[in] xRegister: Register address.
 * @param[in] ulValue: Value to be written.
 */
BaseType_t Eqos_PhyWrite( BaseType_t xAddress, BaseType_t xRegister, uint32_t ulValue )
{
    return EqosHwPhyWrite( xAddress, xRegister, ulValue, &EQOS0 );
}

/**
 * @brief MAC interrupt handler.
 *
 * Handles Tx complete, Rx and recieve buffer unavailable conditions and notifies
 * the handler task.
 *
 * @param pvArg: Unused.
 */
void ETH_SBD_IRQHandler( void * pvArg )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ulStat, ulCh0Stat, ulVal;

    DwEqosDev * pxEthDev = &EQOS0;

    (void) pvArg;

    ulStat = pxEthDev->pxRegs->DMA_STATUS;

    if( ulStat & DMA_STATUS_CHAN0 )
    {
        ulCh0Stat = pxEthDev->pxRegs->DMA_CH0_STATUS;

        pxEthDev->pxRegs->DMA_CH0_STATUS = ulCh0Stat & ( DMA_CHAN_STATUS_NIS | DMA_CHAN_STATUS_RI | DMA_CHAN_STATUS_TI );

        if( ulCh0Stat  & DMA_CHAN_STATUS_RBU )
        {
            /*
             * Rx Buffer Unavailable. Stop Rx for now. Will be restarted after we process the
             * pending frames.
             */

            pxEthDev->pxRegs->DMA_CH0_STATUS = ulCh0Stat & ( DMA_CHAN_STATUS_AIS | DMA_CHAN_STATUS_RBU );
            ulISREvents |= EQOS_RX_EVENT;
            ulRbuFlag = 1;

            ulVal = pxEthDev->pxRegs->MAC_CONFIG;
            ulVal &= ~EQOS_MAC_CONFIG_RE;
            pxEthDev->pxRegs->MAC_CONFIG = ulVal;

            ulVal =  pxEthDev->pxRegs->DMA_CH0_RX_CTRL;
            ulVal &= ~EQOS_DMA_CH0_RX_CONTROL_SR;
            pxEthDev->pxRegs->DMA_CH0_RX_CTRL = ulVal;

            if( xRxHandlerTask != NULL )
            {
                vTaskNotifyGiveFromISR( xRxHandlerTask, &xHigherPriorityTaskWoken );
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }

        if( ulCh0Stat & DMA_CHAN_STATUS_RI )
        {
            ulISREvents |= EQOS_RX_EVENT;

            if( xRxHandlerTask != NULL )
            {
                vTaskNotifyGiveFromISR( xRxHandlerTask, &xHigherPriorityTaskWoken );
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }

        if( ulCh0Stat & DMA_CHAN_STATUS_TI )
        {
            ulISREvents |= EQOS_TX_EVENT;

            if( xRxHandlerTask != NULL )
            {
                vTaskNotifyGiveFromISR( xRxHandlerTask, &xHigherPriorityTaskWoken );
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }
    }

}
/**
 * @brief Discover and initialize the PHY.
 *
 * This function registers the Phy read/write function pointers with the FreeRTOS
 * PHY handling code. After this, calls the FreeRTOS generic/common PHY handling
 * APIs to probe for PHYs connected on the MDIO interface of the MAC and for the
 * PHYs that are found configures/initializes them with the given settings.
 */

void prvPhyInit( void )
{
    vPhyInitialise( &xPhyObject, Eqos_PhyRead, Eqos_PhyWrite );
    xPhyDiscover( &xPhyObject );
    xPhyConfigure( &xPhyObject, &xPHYProperties );
}

/**
 * @brief Initialize Rx/Tx DMA descriptors.
 *
 * @param[in] pxEthDev: Pointer to the DwEqosDev structure.
 */
static void prvInitDescriptors( DwEqosDev * pxEthDev )
{
    pxEthDev->TxDescs = TxDmaDescs;
    pxEthDev->RxDescs = RxDmaDescs;

    pxEthDev->pxRxBufferDescs = RxBufferDescs;

    prvInitRxDescs( pxEthDev );
    prvInitTxDescs( pxEthDev );
}

/**
 * @brief Initialize the Ethernet MAC hardware.
 *
 * @param[in] pxEthDev: Pointer to the DwEqosDev structure.
 */
static BaseType_t prvEqosHwInit( DwEqosDev * pxEthDev )
{
    uint32_t ulVal;
    uint32_t ulTimeout = 100;

    #ifdef EQOS_MTL_HAS_MULTIPLE_QUEUES
        uint32_t ulTxFifoSz, ulRxFifoSz, ulTqs, ulRqs;
    #endif

    /* Soft reset the logic */
    pxEthDev->pxRegs->DMA_BUS_MODE |= DMA_BUS_MODE_SFT_RESET;

    do
    {
        vTaskDelay( pdMS_TO_TICKS( 1 ) );
        ulTimeout--;
    } while( ( pxEthDev->pxRegs->DMA_BUS_MODE & DMA_BUS_MODE_SFT_RESET ) && ulTimeout );

    if ( !ulTimeout )
    {
        return pdFAIL;
    }

    /* Configure MTL Tx Q0 Operating mode */
    pxEthDev->pxRegs->MTL_TXQ0_OP_MODE &= ~MTL_OP_MODE_TXQEN_MASK;
    pxEthDev->pxRegs->MTL_TXQ0_OP_MODE |= MTL_OP_MODE_TXQEN | MTL_OP_MODE_TSF;

    /* Configure MTL RX Q0 operating mode */
    pxEthDev->pxRegs->MTL_RXQ0_OP_MODE |= (MTL_OP_MODE_RSF |
                                    MTL_OP_MODE_FEP |
                                    MTL_OP_MODE_FUP);
    #ifdef EQOS_MTL_HAS_MULTIPLE_QUEUES
        /*
         *  configure the MTL T/Rx Q0 sizes by finding out the configured
         * Tx RX FIFO sizes. Note that this is not needed if the IP is
         * configured to have only one queue (each for Tx and Rx).
         */

        ulVal = dev->pxRegs->MAC_HW_FEATURE_1;

        ulTxFifoSz = ( ulVal >> EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT ) &
                    EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK;

        ulRxFifoSz = ( ulVal >> EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT ) &
                    EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK;

        /*
         * FIFO sizes are encoded as log2(fifo_size) - 7 in the above field
         * and tqs and rqs needs to be programmed in blocks of 256 bytes.
         */

        ulTqs = ( 128 << ulTxFifoSz ) / 256 - 1;
        ulRqs = ( 128 << ulRxFifoSz ) / 256 - 1;

        pxEthDev->pxRegs->MTL_TXQ0_OP_MODE &= ~(MTL_TXQ0_OPERATION_MODE_TQS_MASK <<
                                            MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
        pxEthDev->pxRegs->MTL_TXQ0_OP_MODE |= (tqs <<  MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);

        pxEthDev->pxRegs->MTL_RXQ0_OP_MODE &= ~(EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK <<
                                            MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
        pxEthDev->pxRegs->MTL_RXQ0_OP_MODE |= (rqs << MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    #endif

    /* Enable MAC RXQ */
    pxEthDev->pxRegs->MAC_RXQ_CTRL_0 &= ~(EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK <<
                                        EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);
    pxEthDev->pxRegs->MAC_RXQ_CTRL_0 |= (EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB <<
                                        EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);

    /* Configure RXQ control1 for routing multicast/broadcast queues
     * Note that by default, Q0 will be used.
     */
    pxEthDev->pxRegs->MAC_RXQ_CTRL_1 |= EQOS_MAC_RXQCTRL_MCBCQEN;
    /* Configure tx and rx flow control */

    pxEthDev->pxRegs->MAC_Q0_TX_FLOW_CTRL |= 0xffff << EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT |
                                      EQOS_MAC_Q0_TX_FLOW_CTRL_TFE;
    pxEthDev->pxRegs->MAC_RX_FLOW_CTRL |= EQOS_MAC_RX_FLOW_CTRL_RFE;

    ulVal = pxEthDev->pxRegs->MAC_CONFIG;
    ulVal &= ~EQOS_MAC_CONFIG_JE;
    ulVal |= ( EQOS_MAC_CONFIG_FES | EQOS_MAC_CONFIG_DM );
    #if ( ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM != 0 )
        ulVal |= EQOS_MAC_CONFIG_IPC;
    #endif
    pxEthDev->pxRegs->MAC_CONFIG = ulVal;

    pxEthDev->pxRegs->MAC_PACKET_FILTER |= EQOS_MAC_PACKET_FILTER_PM;
    /* Configure the DMA block */
    pxEthDev->pxRegs->DMA_CH0_TX_CTRL |= (16 << EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

    pxEthDev->pxRegs->DMA_CH0_RX_CTRL |= ((16 << EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT) |
                                   (2048 << EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT));

    ulVal = pxEthDev->pxRegs->DMA_SYS_BUS_MODE;;
    ulVal |= EQOS_DMA_SYSBUS_MODE_BLEN4 | EQOS_DMA_SYSBUS_MODE_BLEN8 |
            EQOS_DMA_SYSBUS_MODE_BLEN16;

    ulVal |= 3 << EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT;
    ulVal |= 1 << EQOS_DMA_SYSBUS_MODE_WR_OSR_LMT_SHIFT;
    ulVal |= EQOS_DMA_SYSBUS_MODE_ONEKBBE;

    pxEthDev->pxRegs->DMA_SYS_BUS_MODE = ulVal;

    prvInitDescriptors( pxEthDev );

    prvSetMacAddr( pxEthDev, ucMACAddress );

    /* Enable DMA Channel 0 interrupts, rx and tx */
    pxEthDev->pxRegs->DMA_CH0_INT_ENABLE |= DMA_CHAN_INTR_ENA_RIE | DMA_CHAN_INTR_ENA_NIE |
        DMA_CHAN_INTR_ENA_TIE | DMA_CHAN_INTR_ENA_AIE | DMA_CHAN_INTR_ENA_RBUE;

    pxEthDev->pxRegs->DMA_CH0_TX_CTRL |= EQOS_DMA_CH0_TX_CONTROL_ST;
    pxEthDev->pxRegs->DMA_CH0_RX_CTRL |= EQOS_DMA_CH0_RX_CONTROL_SR;

    pxEthDev->pxRegs->MAC_CONFIG |= ( EQOS_MAC_CONFIG_RE | EQOS_MAC_CONFIG_TE );

    return pdPASS;
}

/**
 * @brief Update Ethernet MAC speed/duplex settings.
 *
 * Called when a change is observed in the link status and
 * during initial configuration.
 *
 * @param [in] xForce: TRUE/FALSE to force a configuration update.
 */
static void prvEthernetUpdateConfig( BaseType_t xForce )
{
    uint32_t ulDmaChCtrl, ulMacConfig;
    DwEqosDev * pxEthDev = &EQOS0;

    if( ( xForce != pdFALSE ) || ( xPhyObject.ulLinkStatusMask != 0ul ) )
    {
        uint32_t ulConfig;

        ulConfig = pxEthDev->pxRegs->MAC_CONFIG;
        ulConfig &= ~( EQOS_MAC_CONFIG_FES | EQOS_MAC_CONFIG_DM );

        /* Either the initial configuration or the link is up again */
        if( EQOS0.Config.ucAutoNegCtrl == AN_ENABLE )
        {
 		    xPhyStartAutoNegotiation( &xPhyObject, xPhyGetMask( &xPhyObject ) );

            /*
             * Configure the MAC with the Duplex Mode fixed by the
             * auto-negotiation.
             */
            if( xPhyObject.xPhyProperties.ucDuplex == PHY_DUPLEX_FULL )
            {
                ulConfig |= EQOS_MAC_CONFIG_DM;
            }
            else
            {
                /* Half Duplex, Nothing to do here as we have cleared DM in ulConfig */
            }

            /*
             * Configure the MAC with the speed fixed by the
             * auto-negotiation.
             */
            if( xPhyObject.xPhyProperties.ucSpeed == PHY_SPEED_10 )
            {
                /* Nothing to do here as we have FES cleared in ulConfig */
            }
            else
            {
                    /* Speed 100Mbps */
                    ulConfig |= EQOS_MAC_CONFIG_FES;
            }
        }
        else /* AutoNeg Disabled */
        {
            if( EQOS0.Config.ucDuplex == ETH_DUPLEX_HALF )
            {
                xPhyObject.xPhyPreferences.ucDuplex = PHY_DUPLEX_HALF;
            }
            else
            {
                xPhyObject.xPhyPreferences.ucDuplex = PHY_DUPLEX_FULL;
                ulConfig |= EQOS_MAC_CONFIG_DM;
            }

            if( EQOS0.Config.ucSpeed == ETH_SPEED_10M )
            {
                xPhyObject.xPhyPreferences.ucSpeed = PHY_SPEED_10;
            }
            else
            {
                xPhyObject.xPhyPreferences.ucSpeed = PHY_SPEED_100;
                ulConfig |= EQOS_MAC_CONFIG_FES;
            }

            xPhyObject.xPhyPreferences.ucMDI_X = PHY_MDIX_AUTO;

            /* Use fixed configuration. */
            xPhyFixedValue( &xPhyObject, xPhyGetMask( &xPhyObject ) );
        }

        /* Update MAC configuration */
        pxEthDev->pxRegs->MAC_CONFIG = ulConfig;

        /* Start Tx and Rx */
        if( xForce == pdFALSE )
        {
            pxEthDev->pxRegs->DMA_CH0_TX_CTRL |= DMA_CONTROL_ST;
            pxEthDev->pxRegs->DMA_CH0_RX_CTRL |= DMA_CONTROL_SR;
            pxEthDev->pxRegs->MAC_CONFIG |= ( EQOS_MAC_CONFIG_TE | EQOS_MAC_CONFIG_RE );
        }
    }
    else
    {
        /* Stop Tx and Rx */
        ulMacConfig = pxEthDev->pxRegs->MAC_CONFIG;
        ulDmaChCtrl = pxEthDev->pxRegs->DMA_CH0_TX_CTRL;
        ulDmaChCtrl &= ~DMA_CONTROL_ST;
        ulMacConfig &= ~(EQOS_MAC_CONFIG_TE | EQOS_MAC_CONFIG_RE );
        pxEthDev->pxRegs->DMA_CH0_TX_CTRL = ulDmaChCtrl;
        pxEthDev->pxRegs->MAC_CONFIG = ulMacConfig;
        ulDmaChCtrl = pxEthDev->pxRegs->DMA_CH0_RX_CTRL;
        ulDmaChCtrl &= ~DMA_CONTROL_SR;
        pxEthDev->pxRegs->DMA_CH0_RX_CTRL = ulDmaChCtrl;
    }
}

/**
 * @brief Send an Rx Event to the FreeRTOS IP task.
 *
 * @param[in] pxDescriptor: Pointer to the buffer descriptor containing the received packet.
 */
static void prvPassEthMessages( NetworkBufferDescriptor_t * pxDescriptor )
{
    IPStackEvent_t xRxEvent;

    xRxEvent.eEventType = eNetworkRxEvent;
    xRxEvent.pvData = ( void * ) pxDescriptor;

    if( xSendEventStructToIPTask( &xRxEvent, ( TickType_t ) 1000 ) != pdPASS )
    {
        vReleaseNetworkBufferAndDescriptor( pxDescriptor );
        iptraceETHERNET_RX_EVENT_LOST();
        FreeRTOS_printf( ( "prvPassEthMessages: Can not queue packet!\n" ) );
    }
    else
    {
        iptraceNETWORK_INTERFACE_RECEIVE();
    }
}

/**
 * @brief Recieve a packet from the DMA buffers.
 *
 */
static BaseType_t prvNetworkInterfaceInput( void )
{
    const TickType_t xDescriptorWaitTime = pdMS_TO_TICKS( niDESCRIPTOR_WAIT_TIME_MS );
    uint32_t ulCurIdx, ulVal, ulCh0Stat;
    DwEqosDev * pxEthDev = &EQOS0;
    DmaDesc * pxDesc;
    BaseType_t xReceivedLength;
    NetworkBufferDescriptor_t * pxNewDescriptor = NULL, *pxCurDescriptor;
    uint8_t * pucBuffer;

    ulCurIdx = pxEthDev->ulCurRxDescId;
    pxDesc = &pxEthDev->RxDescs[ ulCurIdx ];
    SCB_InvalidateDCache_by_Addr( pxDesc, sizeof( DmaDesc ) );

    while( ! ( pxDesc->ulDes3 & RDES3_OWN ) )
	{
        ulCurIdx = pxEthDev->ulCurRxDescId;
        xReceivedLength = ( pxDesc->ulDes3 & 0x7fff ) - 4;

        #if ( ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM != 0 )
            if ( ( pxDesc->ulDes1 & RDES1_IP_HDR_ERROR ) || ( pxDesc->ulDes1 & RDES1_IP_CSUM_ERROR ) )
            {
                /* There are checksum errors in this frame, do not forward to IP, just refresh the descriptor */
                #if ( ipconfigZERO_COPY_RX_DRIVER != 0 )
                    /* Find out the NetworkDescriptor associated with this DMA descriptor */
                    pxCurDescriptor = pxEthDev->pxRxBufferDescs[ ulCurIdx ];
                    pxDesc->ulDes0 = ( uint32_t ) LocalToGlobal( pxCurDescriptor->pucEthernetBuffer );
                #else
                    pxDesc->ulDes0 = ( uint32_t ) LocalToGlobal( &RxBuffers[ ulCurIdx ][ 0 ] );
                #endif
                pxDesc->ulDes3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN | RDES3_BUFFER1_VALID_ADDR;

                SCB_CleanDCache_by_Addr( ( uint32_t * ) pxDesc, sizeof( DmaDesc ) );

                pxEthDev->pxRegs->DMA_CH0_RX_END_ADDR = ( uint32_t ) LocalToGlobal( &( pxEthDev->RxDescs[ ulCurIdx ] ) );
                pxEthDev->ulCurRxDescId++;
                pxEthDev->ulCurRxDescId %= RX_DESC_COUNT;
                pxDesc = &pxEthDev->RxDescs[ pxEthDev->ulCurRxDescId ];

                /* continue processing any pending DMA descriptors */
                continue;
            }
        #endif

        pxNewDescriptor = pxGetNetworkBufferWithDescriptor( ETH_BUF_SIZE, xDescriptorWaitTime );

        if ( pxNewDescriptor != NULL )
        {
            #if ( ipconfigZERO_COPY_RX_DRIVER != 0 )
                /* Find out the NetworkDescriptor associated with this DMA descriptor */
                pxCurDescriptor = pxEthDev->pxRxBufferDescs[ ulCurIdx ];
                SCB_InvalidateDCache_by_Addr( pxCurDescriptor->pucEthernetBuffer, ETH_BUF_SIZE );
                configASSERT( pxCurDescriptor != NULL );
            #else
                pucBuffer = &RxBuffers[ ulCurIdx ][ 0 ];
                SCB_InvalidateDCache_by_Addr( pucBuffer, ETH_BUF_SIZE );
                pxCurDescriptor = pxNewDescriptor;
                memcpy( pxNewDescriptor->pucEthernetBuffer, pucBuffer, xReceivedLength );
            #endif

            pxCurDescriptor->xDataLength = xReceivedLength;

            prvPassEthMessages( pxCurDescriptor );
        }
        else
        {
            /* Buffer Allocation failed, use the old buffer to refresh the descriptor
             * if zero copy Rx is being used.
             */
            #if ( ipconfigZERO_COPY_RX_DRIVER != 0)
                pxNewDescriptor = pxEthDev->pxRxBufferDescs[ ulCurIdx ];
            #endif
        }

        /* refresh the descriptor */
        #if ( ipconfigZERO_COPY_RX_DRIVER != 0)
            pxDesc->ulDes0 = ( uint32_t ) LocalToGlobal( pxNewDescriptor->pucEthernetBuffer );
            pxEthDev->pxRxBufferDescs[ ulCurIdx ] = pxNewDescriptor;
        #else
            pxDesc->ulDes0 = ( uint32_t ) LocalToGlobal( &RxBuffers[ ulCurIdx ][ 0 ] );
        #endif
        pxDesc->ulDes3 = RDES3_OWN | RDES3_INT_ON_COMPLETION_EN |
                            RDES3_BUFFER1_VALID_ADDR;

        SCB_CleanDCache_by_Addr( ( uint32_t * ) &pxEthDev->RxDescs[ ulCurIdx ], sizeof( DmaDesc ) );

        pxEthDev->pxRegs->DMA_CH0_RX_END_ADDR = ( uint32_t ) LocalToGlobal( &( pxEthDev->RxDescs[ ulCurIdx ] ) );
        pxEthDev->ulCurRxDescId++;
        pxEthDev->ulCurRxDescId %= RX_DESC_COUNT;
        pxDesc = &pxEthDev->RxDescs[ pxEthDev->ulCurRxDescId ];
    }


    if (ulRbuFlag)
    {
        /* Restart Rx */
        pxEthDev->pxRegs->DMA_CH0_RX_CTRL |= EQOS_DMA_CH0_RX_CONTROL_SR;
        pxEthDev->pxRegs->MAC_CONFIG |= EQOS_MAC_CONFIG_RE;

        ulRbuFlag = 0;
    }

    return pdPASS;
}

static uint32_t ulTxDescClearIdx;

/**
 * @brief Free the Network buffer and descriptor used for Tx.
 *
 * Free as many Network buffers and descriptor as possible used during Tx if zero copy
 * Tx is being used. Release a Tx Descritptor semaphore for each free (non-'OWN'ed)
 * DMA descriptor.
 */
static void vClearTXBuffers()
{
    DmaDesc *pxDesc;
    DwEqosDev *pxEthDev = &EQOS0;

    #if ( ipconfigZERO_COPY_TX_DRIVER != 0 )
        NetworkBufferDescriptor_t * pxNetworkBuffer;
        uint8_t * ucPayLoad;
    #endif

    size_t uxCount = ( ( UBaseType_t ) TX_DESC_COUNT ) - uxSemaphoreGetCount( xTXDescriptorSemaphore );
    uint32_t ulCurIdx = pxEthDev->ulCurTxDescId;
    pxDesc = &pxEthDev->TxDescs[ ulTxDescClearIdx ];

    while( ( uxCount > 0 ) && ( ( pxDesc->ulDes3 & TDES3_OWN ) == 0 ) )
	{
        if (ulTxDescClearIdx == ulCurIdx)
            break;
        #if ( ipconfigZERO_COPY_TX_DRIVER != 0 )
        {
            ucPayLoad = ( uint8_t * ) GlobalToLocal( ( void * ) pxDesc->ulDes0 );

            if( ucPayLoad != NULL )
            {
                pxNetworkBuffer = pxPacketBuffer_to_NetworkBuffer( ucPayLoad );

                if( pxNetworkBuffer != NULL )
                {
                    vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
                }
            }
        }
        #endif /* ipconfigZERO_COPY_TX_DRIVER */

        xSemaphoreGive( xTXDescriptorSemaphore );
        uxCount--;
        ulTxDescClearIdx++;
        ulTxDescClearIdx %= TX_DESC_COUNT;
        pxDesc = &pxEthDev->TxDescs[ ulTxDescClearIdx ];
    }
}

/**
 * @brief Task function that handles all the events signalled by the interrupt handler.
 *
 * The task processes the Rx and Tx complete events signalled by the DMA interrupts.
 * It also monitors the resource usage statistics and the link status.
 *
 * @param[in] pvParameters: Unused
 */
static void prvEqosHandlerTask( void * pvParameters )
{
    UBaseType_t uxCurrentCount;
    BaseType_t xResult;
    const TickType_t ulMaxBlockTime = pdMS_TO_TICKS( 50UL );

    ( void ) pvParameters;

    for( ; ; )
    {
        xResult = 0;

        #if ( ipconfigHAS_PRINTF != 0 )
        {
            /* Call a function that monitors resources: the amount of free
             * network buffers and the amount of free space on the heap.  See FreeRTOS_IP.c
             * for more detailed comments.
             */
            vPrintResourceStats();
        }
        #endif /* ( ipconfigHAS_PRINTF != 0 ) */

        if( xTXDescriptorSemaphore != NULL )
        {
            static UBaseType_t uxLowestSemCount = ( UBaseType_t ) TX_DESC_COUNT - 1;

            uxCurrentCount = uxSemaphoreGetCount( xTXDescriptorSemaphore );

            if( uxLowestSemCount > uxCurrentCount )
            {
                uxLowestSemCount = uxCurrentCount;
                FreeRTOS_printf( ( "TX DMA buffers: lowest %lu\n", uxLowestSemCount ) );
            }
        }

        if( ( ulISREvents & EQOS_ALL_EVENT ) == 0 )
        {
            /* No events to process now, wait for the next. */
            ulTaskNotifyTake( pdFALSE, ulMaxBlockTime );
        }

        if( ( ulISREvents & EQOS_RX_EVENT ) != 0 )
        {

            ulISREvents &= ~EQOS_RX_EVENT;

            xResult = prvNetworkInterfaceInput();
        }

        if( ( ulISREvents & EQOS_TX_EVENT ) != 0 )
        {
            ulISREvents &= ~EQOS_TX_EVENT;
	        /* Release the semaphore, also free the NetworkDescriptor if ZERO_COPY_TX is used */
            vClearTXBuffers();
        }

        if( xPhyCheckLinkStatus( &xPhyObject, xResult ) != 0 )
        {
            /* Linkstatus has changed, update configuration */
            prvEthernetUpdateConfig( pdFALSE );
        }
    }
}

static BaseType_t prvPinMuxConfigure( void )
{
    int32_t ret;
    uint32_t val;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_0, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_2, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_3, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_5, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_6, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_7, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_8, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_9, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    ret = PINMUX_Config( PORT_NUMBER_1, PIN_NUMBER_10, PINMUX_ALTERNATE_FUNCTION_3 );
    if ( ret )
        return pdFAIL;

    /* program the clock selection mux to select phy clk as the 50M clock */
    val = *( ( volatile unsigned int * ) ETH_50M_CLK_MUX_REG );
    val |= SEL_PHY_REFCLK;
    *( ( volatile unsigned int * ) ETH_50M_CLK_MUX_REG ) = val;

    return pdPASS;
}

/**
 * @brief Initialise the Network Interface.
 *
 * Initialise the MAC hardware and DMA descriptors. Also, probe for and configure
 * the PHYs that are available in the system. Create the FreeRTOS task that handles
 * the events signalled by the interrupt handler and create the counting semaphore
 * used to synchronize/manage the Tx DMA descriptor usage.
 *
 * @return pdPASS, if HW initialisation was successful and a link could be detected. Else pdFALSE.
 */
BaseType_t xNetworkInterfaceInitialise( void )
{
    BaseType_t xReturn = pdPASS;
    static uint8_t ucHwInitDone = 0;

    if( !ucHwInitDone )
    {
	    xReturn = prvPinMuxConfigure();

        if ( xReturn != pdPASS )
            return xReturn;

        xReturn = prvEqosHwInit( &EQOS0 );

        if ( xReturn != pdPASS )
            return xReturn;

        prvPhyInit();

        ucHwInitDone = 1;
    }

    prvEthernetUpdateConfig( pdTRUE );

    if( xRxHandlerTask == NULL )
        xReturn = xTaskCreate( prvEqosHandlerTask, "EQOS_MAC", HANDLER_TASK_STACK_SIZE, NULL,
                                                configMAX_PRIORITIES - 1, &xRxHandlerTask );

    if( xTXDescriptorSemaphore == NULL )
        xTXDescriptorSemaphore = xSemaphoreCreateCounting( ( UBaseType_t ) TX_DESC_COUNT,
                                                                 ( UBaseType_t ) TX_DESC_COUNT );

    /* Set priority and enable interrupts */
    NVIC_SetPriority( EQOS0.xIrq, 2 );
    NVIC_EnableIRQ( EQOS0.xIrq );


    if( xPhyObject.ulLinkStatusMask != 0ul )
    {
        xReturn = pdPASS;
    }
    else
    {
        xReturn = pdFAIL;
    }

    return xReturn;
}

/**
 * @brief Transmit a network packet.
 *
 * @param[in] pxNetworkBuffer: Descriptor containing the packet to be sent.
 * @param[in] xReleaseAfterSend: TRUE if the buffer and descriptor needs to be freed after Tx. Else FALSE.
 * @return pdPASS, if the transmission was successful. Else pdFALSE.
 */
BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxNetworkBuffer,
                                        BaseType_t xReleaseAfterSend )
{
    BaseType_t xReturn = pdFAIL;
    uint32_t ulTransmitSize = 0, ulCurIdx;
    DmaDesc *pxDesc;
    DwEqosDev *pxEthDev = &EQOS0;

    /* maximum time to block for a free Tx descriptor */
    const TickType_t xBlockTimeTicks = pdMS_TO_TICKS( 50u );

    do {
        /* make sure we have link */
        if ( xPhyObject.ulLinkStatusMask != 0ul )
        {
            if( xSemaphoreTake( xTXDescriptorSemaphore, xBlockTimeTicks ) != pdPASS )
            {
                /* Time-out waiting for a free TX descriptor. */
                break;
			}

            ulCurIdx = pxEthDev->ulCurTxDescId;
            pxDesc = &pxEthDev->TxDescs[ ulCurIdx ];

            ulTransmitSize = pxNetworkBuffer->xDataLength;

            if ( ulTransmitSize > ETH_BUF_SIZE )
            {
                ulTransmitSize = ETH_BUF_SIZE;
            }

            pxDesc->ulDes2 = TDES2_INTERRUPT_ON_COMPLETION | ulTransmitSize;
            #if ( ipconfigZERO_COPY_TX_DRIVER == 0 )
            {
                memcpy( ( void * ) &TxBuffers[ ulCurIdx ][ 0 ], pxNetworkBuffer->pucEthernetBuffer, ulTransmitSize );
                pxDesc->ulDes0 = ( uint32_t ) LocalToGlobal( &TxBuffers[ ulCurIdx ][ 0 ] );
                SCB_CleanDCache_by_Addr( ( uint32_t * ) &TxBuffers[ ulCurIdx ][ 0 ], ETH_BUF_SIZE);
            }
            #else
            {
                configASSERT( xReleaseAfterSend != 0 );
                pxDesc->ulDes0 = ( uint32_t ) LocalToGlobal( pxNetworkBuffer->pucEthernetBuffer );
                SCB_CleanDCache_by_Addr( ( uint32_t * ) pxNetworkBuffer->pucEthernetBuffer, ETH_BUF_SIZE);

                /* The Buffer has been taken over by the DMA, do not release it */
                xReleaseAfterSend = pdFALSE_UNSIGNED;
            }
            #endif


            pxEthDev->ulCurTxDescId++;
            pxEthDev->ulCurTxDescId %= TX_DESC_COUNT;

            pxDesc->ulDes3 = TDES3_OWN | TDES3_LAST_DESCRIPTOR | TDES3_FIRST_DESCRIPTOR |
                            ulTransmitSize;
            #if ( ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM != 0 )
                pxDesc->ulDes3 |= (TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT);
            #endif

            SCB_CleanDCache_by_Addr( ( uint32_t * ) pxDesc, sizeof( DmaDesc ) );

            pxEthDev->pxRegs->DMA_CH0_TX_END_ADDR = ( uint32_t ) LocalToGlobal( &( pxEthDev->TxDescs[ pxEthDev->ulCurTxDescId ] ) );
            xReturn = pdPASS;
        }
        else
        {
			/* There is no link, drop the packet */
        }
    } while( 0 );


    if( xReleaseAfterSend != pdFALSE )
    {
        vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
    }

    return xReturn;
}

/*-----------------------------------------------------------*/
/* Used with BufferAllocation_1 strategy */
#define NUMBUF	ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS
/**@brief Buffer area used with BufferAllocation_1.c */
static uint8_t ucNetworkPackets[ NUMBUF * ETH_BUF_SIZE ] __attribute__((aligned(32))) __attribute__((section("eth_buf")));
/**
 * @brief Allocate packet buffer area to descriptors. Used with BufferAllocation_1 strategy.
 *
 * @param[in] pxNetworkBuffers: Array of NetworkBuffer descriptors.
 */
void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] )
{

    uint8_t * ucRAMBuffer = ucNetworkPackets;
    uint32_t ul;

    for( ul = 0; ul < NUMBUF; ul++ )
    {
        pxNetworkBuffers[ ul ].pucEthernetBuffer = ucRAMBuffer + ipBUFFER_PADDING;
        *( ( unsigned * ) ucRAMBuffer ) = ( unsigned ) ( &( pxNetworkBuffers[ ul ] ) );
        ucRAMBuffer += ETH_BUF_SIZE;
    }
}
/*-----------------------------------------------------------*/
