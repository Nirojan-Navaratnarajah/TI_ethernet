//#############################################################################
//
// FILE:   fsi_ex8_ext_p2pconnection_rx.c
//
// TITLE:  FSI Receive in Point to Point Connection
//
//! \addtogroup driver_example_list
//! <h1>FSI P2Point Connection:Rx Side</h1>
//!
//! Example sets up FSI receiving device in a point to point connection
//! to the FSI transmitting device. Example code to set up FSI transmit device
//! is implemented in a separate file.
//!
//! In a real scenario two separate devices may power up in arbitrary order and
//! there is a need to establish a clean communication link which ensures that
//! receiver side is flushed to properly interpret the start of a new valid
//! frame.
//!
//! There is no true concept of a master or a slave node in the FSI protocol,
//! but to simplify the data flow and connection we can consider transmitting
//! device as master and receiving side as slave. Transmitting side will be
//! driver of initialization sequence.
//!
//! Handshake mechanism which must take place before actual data transmission
//! can be usecase specific; points described below can be taken as an example
//! on how to implement the handshake from receiving side -
//!
//! - Setup the receiver interrupts to detect PING type frame reception
//! - Begin the first PING loop
//!       + Wait for receiver interrupt
//!       + If the FSI Rx has received a PING frame with \b FSI_FRAME_TAG0, come
//!         out of loop. Otherwise iterate the loop again.
//! - Begin the second PING loop
//!       + Send the Flush sequence
//!       + Send the PING frame with tag \bFSI_FRAME_TAG1
//!       + Wait for receiver interrupt
//!       + If the FSI Rx has received a PING frame with \b FSI_FRAME_TAG1, come
//!         out of loop. Otherwise iterate the loop again.
//!  - Now, the receiver side has received the acknowledged PING frame(tag1), so
//!    it is ready for normal operation further.
//!
//! After above synchronization steps, FSI Rx can be configured as per usecase
//! i.e. nWords, lane width, enabling events etc and start the infinite
//! transfers. More details on establishing the communication link can be found
//! in device TRM.
//!
//! User can edit some of configuration parameters as per usecase, similar to
//! other examples.
//!
//! \b nWords - Number of words per transfer may be from 1 -16
//! \b nLanes - Choice to select single or double lane for frame transfers
//! \b fsiClock - FSI Clock used for transfers
//! \b txUserData - User data to be sent with Data frame
//! \b txDataFrameTag - Frame tag used for Data transfers
//! \b txPingFrameTag - Frame tag used for Ping transfers
//! \b txPingTimeRefCntr - Tx Ping timer reference counter
//! \b rxWdTimeoutRefCntr - Rx Watchdog timeout reference counter
//!
//!\b External \b Connections \n
//!   For FSI external P2P connection, external connections are required 
//!   to be made between two devices. Device 1's FSI TX and RX pins need to
//!   be connected to device 2's FSI RX and TX pins respectively. See below 
//!	  for external connections to make and GPIOs used:
//!
//!	 External connections required between independent RX and TX devices:	
//!  - FSIRX_CLK to FSITX_CLK
//!  - FSIRX_RX0 to FSITX_TX0
//!  - FSIRX_RX1 to FSITX_TX1 
//!
//!  ControlCard FSI Header GPIOs:
//!  - GPIO_27  ->    FSITXA_CLK
//!  - GPIO_26  ->    FSITXA_TX0
//!  - GPIO_25  ->    FSITXA_TX1
//!  - GPIO_9  ->     FSIRXA_CLK
//!  - GPIO_8  ->     FSIRXA_RX0
//!  - GPIO_10  ->    FSIRXA_RX1
//!
//! \b Watch \b Variables \n
//!  - \b dataFrameCntr  Number of Data frame received
//!  - \b error          Non zero for transmit/receive data mismatch
//!



// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "HellaDefinitions.h"
#include "FsiRegisterDefinitions.h"
#include "ipc.h"

//
// Defines
//
#define IPC_CMD_READ_MEM   0x1001
#define IPC_CMD_RESP       0x2001

#define TEST_PASS          0x5555
#define TEST_FAIL          0xAAAA

#define PACKET_LENGTH 132
#pragma DATA_SECTION(packetData, "MSGRAM_CPU_TO_CM")
uint8_t packetData[PACKET_LENGTH];

uint32_t pass;


/* Register Definitions for FSI Tx-Regs */
#pragma DATA_SECTION(FSI_Sys_FsiTxaRegs_st,"FsiTxaRegs");
volatile struct ST_FSI_TX_REGS FSI_Sys_FsiTxaRegs_st;

/* Register Definitions for FSI Rx-Regs */
#pragma DATA_SECTION(FSI_Sys_FsiRxaRegs_st,"FsiRxaRegs");
volatile struct ST_FSI_RX_REGS FSI_Sys_FsiRxaRegs_st;





int i = 0;
    volatile uint32_t mode = 0U;
    uint32_t dataBytes = 64;





#define PRESCALER_VAL    2U

//
// Globals, User can modify these parameters as per usecase
//
// Number of words per transfer may be from 1 -16
uint16_t nWords = 16;

// Transfer can be happen over single or double lane
FSI_DataWidth nLanes = FSI_DATA_WIDTH_2_LANE;


// Frame tag used with Data/Ping transfers
FSI_FrameTag txDataFrameTag = FSI_FRAME_TAG10, txPingFrameTag = FSI_FRAME_TAG15;


// Boolean flag to enable/disable Rx Frame Watchdog
bool isRxFrameWdEnable = true;

//
// This value can be anything suitable to generate a single interrupt event,
// lower values may lead WD to trigger another event even before handler of 1st
// one is not completed
//
uint32_t rxFrameWdRefCntr = 0x1000000;

//
// Globals, these are not config parameters, user are not required to edit them
//
uint16_t txEventSts = 0, rxEventSts = 0;
uint16_t *txBufAddr = 0, *rxBufAddr = 0;


uint32_t dataFrameCntr = 0;

uint32_t error_fsi = 0;

void initFSI(void);

void init_CM_for_ethernet(void);

void sync_cores(void);
//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();



    init_CM_for_ethernet();

    //
    // Initialize basic settings for FSI
    //
    initFSI();


    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;


    DEVICE_DELAY_US(10);

    FSI_setRxSoftwareFrameSize(FSIRXA_BASE, nWords);
    FSI_setRxDataWidth(FSIRXA_BASE, nLanes);


    int i;

    //clear the data recieved flag
  //  FSI_Sys_FsiRxaRegs_st.RX_EVT_CLR_un.bit_st.DATA_FRAME_bt1=1;


  //  txMsg[loopCnt].data[1] = (FSI_Sys_FsiRxaRegs_st.RX_BUF_BASE_au16[0] & 0x00FFU )  ;

    sync_cores();


    while(1)
    {

        int j=0;
        //
        //Form the unicast Ethernet Packet in Memory
        //
        for(i=0;i<PACKET_LENGTH/4;i++)
        {
            //
            //First 6 bytes of the packet are the MAC Destination Address
            //Bytes, the Destination and CRC shall be inserted by the hardware
            //
            if(i == 0)
                *((uint32_t *)packetData + i) = 0x01020304;
            else if(i == 1)
                *((uint32_t *)packetData + i)  = 0xFFFF0506;
            else
            { // HWREG((uint32_t *)packetData +i) = 0xAAAA;

              //  HWREG((uint32_t *)packetData +i) = FSI_Sys_FsiRxaRegs_st.RX_BUF_BASE_au16[j];
                //j++

               // HWREG((uint32_t *)packetData + i) = (uint32_t)FSI_Sys_FsiRxaRegs_st.RX_BUF_BASE_au16[j];
             //   HWREG((uint32_t *)packetData + i) = ((uint32_t)FSI_Sys_FsiRxaRegs_st.RX_BUF_BASE_au16[j] << 16) | (uint32_t)FSI_Sys_FsiRxaRegs_st.RX_BUF_BASE_au16[j + 1];
                HWREG((uint32_t *)packetData + i) =((uint32_t)FSI_Sys_FsiRxaRegs_st.RX_BUF_BASE_au16[j + 1] << 16) | (uint32_t)FSI_Sys_FsiRxaRegs_st.RX_BUF_BASE_au16[j];
                j=j+2;


            }
        }


        //
        // Send a message without message queue
        // Since C28x and CM does not share the same address space for shared RAM,
        // ADDRESS_CORRECTION is enabled
        // Length of the data to be read is passed as data.
        //
    //    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
     //   IPC_CMD_READ_MEM, (uint32_t)packetData, PACKET_LENGTH);


        //
        // Check whether the flags are not busy
        //
        if((IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_FLG & IPC_FLAG0) == 0U)
        {



        //
        // Update the command registers. ADDR register holds the offset
        // from the base address of the MSG RAM
        //
        IPC_Instance[IPC_CPU1_L_CM_R].IPC_SendCmd_Reg->IPC_SENDCOM  = IPC_CMD_READ_MEM;
        IPC_Instance[IPC_CPU1_L_CM_R].IPC_SendCmd_Reg->IPC_SENDDATA = PACKET_LENGTH;
        IPC_Instance[IPC_CPU1_L_CM_R].IPC_SendCmd_Reg->IPC_SENDADDR =
            (uint32_t)packetData - IPC_Instance[IPC_CPU1_L_CM_R].IPC_MsgRam_LtoR;



        //
        // Set the flags to indicate the remote core
        //
        IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_SET = IPC_FLAG0;

        }




        //
        // Wait for acknowledgment
        //

        //  IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);

        while((IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_FLG & IPC_FLAG0) != 0U)
            {
            }



        //
        // Read response
        //
        if(IPC_getResponse(IPC_CPU1_L_CM_R) == TEST_PASS)
        {
            pass = 1;
        }
        else
        {
            pass = 0;
        }

        DEVICE_DELAY_US(5);

        //
        // End of example. Loop forever
        //

    }

        //
        // End of File
        //


}



void init_CM_for_ethernet(void){


            //
            // Boot CM core
            //
        #ifdef _FLASH
            Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
        #else
            Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
        #endif

            //
            // Set up EnetCLK to use SYSPLL as the clock source and set the
            // clock divider to 2.
            //
            // This way we ensure that the PTP clock is 100 MHz. Note that this value
            // is not automatically/dynamically known to the CM core and hence it needs
            // to be made available to the CM side code beforehand.
            SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);

            //
            // Configure the GPIOs for ETHERNET.
            //

            //
            // MDIO Signals
            //
            GPIO_setPinConfig(GPIO_105_ENET_MDIO_CLK);
            GPIO_setPinConfig(GPIO_106_ENET_MDIO_DATA);

            //
            // Use this only for RMII Mode
            //GPIO_setPinConfig(GPIO_73_ENET_RMII_CLK);
            //

            //
            //MII Signals
            //
            GPIO_setPinConfig(GPIO_109_ENET_MII_CRS);
            GPIO_setPinConfig(GPIO_110_ENET_MII_COL);

            GPIO_setPinConfig(GPIO_75_ENET_MII_TX_DATA0);
            GPIO_setPinConfig(GPIO_122_ENET_MII_TX_DATA1);
            GPIO_setPinConfig(GPIO_123_ENET_MII_TX_DATA2);
            GPIO_setPinConfig(GPIO_124_ENET_MII_TX_DATA3);

            //
            //Use this only if the TX Error pin has to be connected
            //GPIO_setPinConfig(GPIO_46_ENET_MII_TX_ERR);
            //

            GPIO_setPinConfig(GPIO_118_ENET_MII_TX_EN);

            GPIO_setPinConfig(GPIO_114_ENET_MII_RX_DATA0);
            GPIO_setPinConfig(GPIO_115_ENET_MII_RX_DATA1);
            GPIO_setPinConfig(GPIO_116_ENET_MII_RX_DATA2);
            GPIO_setPinConfig(GPIO_117_ENET_MII_RX_DATA3);
            GPIO_setPinConfig(GPIO_113_ENET_MII_RX_ERR);
            GPIO_setPinConfig(GPIO_112_ENET_MII_RX_DV);

            GPIO_setPinConfig(GPIO_44_ENET_MII_TX_CLK);
            GPIO_setPinConfig(GPIO_111_ENET_MII_RX_CLK);

            //
            //Power down pin to bring the external PHY out of Power down
            //
            GPIO_setDirectionMode(108, GPIO_DIR_MODE_OUT);
            GPIO_setPadConfig(108, GPIO_PIN_TYPE_PULLUP);
            GPIO_writePin(108,1);

            //
            //PHY Reset Pin to be driven High to bring external PHY out of Reset
            //

            GPIO_setDirectionMode(119, GPIO_DIR_MODE_OUT);
            GPIO_setPadConfig(119, GPIO_PIN_TYPE_PULLUP);
            GPIO_writePin(119,1);


}

//
// initFSI - Initializes FSI Tx/Rx and also sends FLUSH sequence.
//
void initFSI(void)
{
    FSI_disableRxInternalLoopback(FSIRXA_BASE);

    //
    // NOTE: External loopback, Modify GPIO settings as per setup
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_TXCLK);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_TX0);

    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_RXCLKA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_RX0A);
    if(nLanes == FSI_DATA_WIDTH_2_LANE)
    {
		GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_TX1);
        GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_RX1A);
    }

    //
    // Set RX GPIO to be asynchronous
    // (pass through without delay)
    // Default setting is to have 2 SYS_CLK cycles delay
    //
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_FSI_RX0A, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_FSI_RXCLKA, GPIO_QUAL_ASYNC);
    if(nLanes == FSI_DATA_WIDTH_2_LANE)
    {
        GPIO_setQualificationMode(DEVICE_GPIO_PIN_FSI_RX1A, GPIO_QUAL_ASYNC);
    }

    FSI_performTxInitialization(FSITXA_BASE, PRESCALER_VAL);
    FSI_performRxInitialization(FSIRXA_BASE);
    txBufAddr = (uint16_t *)FSI_getTxBufferAddress(FSITXA_BASE);
    rxBufAddr = (uint16_t *)FSI_getRxBufferAddress(FSIRXA_BASE);
}


void sync_cores(){

        //
        // Clear any IPC flags if set already
        //
        //  IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);
        IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_CLR = IPC_FLAG_ALL;
        //
        // Synchronize both the cores.
        //
        //  IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);
        // IPC_setFlagLtoR(ipcType, flag);
        IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_SET = IPC_FLAG31;

        //  IPC_waitForFlag(ipcType, flag);
        while((IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_STS & IPC_FLAG31) == 0U)
        {
        }
        //   IPC_ackFlagRtoL(ipcType, flag);
        IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_ACK = IPC_FLAG31;

        // IPC_waitForAck(ipcType, flag);
        while((IPC_Instance[IPC_CPU1_L_CM_R].IPC_Flag_Ctr_Reg->IPC_FLG & IPC_FLAG31) != 0U)
        {
        }


}



//
// End of File
//
