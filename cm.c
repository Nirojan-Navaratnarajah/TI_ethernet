//#############################################################################
//
// FILE:   ethernet_ipc_ex1_basic_cm.c
//
// TITLE:  Ethernet + IPC example with interrupt
//
//! \addtogroup driver_cm_c28x_dual_example_list
//! <h1> Ethernet + IPC basic message passing example with interrupt </h1>
//!
//! This example demonstrates how to receive the message passed from C28x side
//! containing the Ethernet Packet over IPC,sends the packet on Ethernet 
//! acknowledge the packet received over IPC to C28x core. 
//! This extends the IPC Example. Uses IPC C28x to CM core 
//! without message queues. It can be used as a reference for flow using Ethernet 
//! and IPC together. If the Packet data received over Ethernet is to be passed
//! to C28x for control applications, it can send IPC message to C28x. 
//! It is recommended to run the C28x1 core first, followed by the CM core.
//! The example actually uses the internal loopback mode of MAC hence the MII
//! Tx and Rx signals are not used, but needs the MII Tx and Rx Clock signals
//! which comes from the External PHY.
//!
//! \b External \b Connections \n
//!  - MII connections on Control card
//!
//! \b Watch \b Variables \n
//!  - None.
//!
//
//#############################################################################
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "driverlib_cm.h"
#include "ipc.c"
//#include "ethernet.h"
//#include "ethernet.c"
//
// Defines
//
#define IPC_CMD_READ_MEM   0x1001
#define IPC_CMD_RESP       0x2001

#define TEST_PASS          0x5555
#define TEST_FAIL          0xAAAA

//
// Defines
//
#define PACKET_LENGTH 132


#define ETHERNET_NO_OF_RX_PACKETS   1U
//
//Change this define for changing Packet buffer length
//
#define ETHERNET_MAX_PACKET_LENGTH 1538U



//
// Globals
//
uint8_t pData[PACKET_LENGTH];
uint8_t Ethernet_rxBuffer[ETHERNET_NO_OF_RX_PACKETS *
                          ETHERNET_MAX_PACKET_LENGTH];
Ethernet_Handle emac_handle;
Ethernet_Pkt_Desc pktDesc;
extern uint32_t Ethernet_rxInterruptCount;
bool status = false;

//
// IPC ISR for Flag 0.
// C28x core sends data without message queue using Flag 0
//
void IPC_ISR0()
{
    uint32_t command, addr, data;


    //
    // Read the command
    //
  //  IPC_readCommand(IPC_CM_L_CPU1_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
   //                 &command, &addr, &data);


    uint32_t addrReg;

    //
    // Check whether the flags are not empty
    //
    if((IPC_Instance[IPC_CM_L_CPU1_R].IPC_Flag_Ctr_Reg->IPC_STS & IPC_FLAG0) != 0U)
    {

        //
        // Read the command registers
        //
        command   = IPC_Instance[IPC_CM_L_CPU1_R].IPC_RecvCmd_Reg->IPC_RECVCOM;
        addrReg   = IPC_Instance[IPC_CM_L_CPU1_R].IPC_RecvCmd_Reg->IPC_RECVADDR;
        data      = IPC_Instance[IPC_CM_L_CPU1_R].IPC_RecvCmd_Reg->IPC_RECVDATA;


        //
        // Calculate the address form the offset
        //
        addr = IPC_Instance[IPC_CM_L_CPU1_R].IPC_MsgRam_RtoL +
                IPC_ADDR_OFFSET_CORR(addrReg,
                                IPC_Instance[IPC_CM_L_CPU1_R].IPC_Offset_Corr);

    }

    if(command == IPC_CMD_READ_MEM)
    {
        status = true;
        //
        //Read the buffer address and Length
        //from the IPC message
        //
        pktDesc.dataBuffer = (uint8_t *)addr;
        pktDesc.bufferLength = data;
        pktDesc.pktLength = data;
        pktDesc.validLength = data;
        //
        //Send the packet on receiving Message from C28x side
        //
        Ethernet_sendPacket(emac_handle,&pktDesc);

/*
        uint32_t error_code_ethernet = ETHERNET_RET_SUCCESS;
        Ethernet_Device *devicePtr;
        uint32_t fragcnt, pktlen;
        Ethernet_Pkt_Desc *pktLastPtr;
        Ethernet_DescCh *channelDescPtr;
        Ethernet_DmaObj *dmaObj;
        Ethernet_ChInfo *txChan;

        devicePtr = (Ethernet_Device *)emac_handle;

        if ((NULL == devicePtr) || (devicePtr->devMagic != ETHERNET_DRV_DEVMAGIC)) {
            error_code_ethernet = ETHERNET_ERR_INVALID_HANDLE;
        }

        if (NULL == pktDesc) {
            error_code_ethernet = ETHERNET_ERR_INVALID_PARAM;
        } else {
            // Do some packet validation
            if (0U == (pktDesc->flags & ETHERNET_PKT_FLAG_SOP)) {
                error_code_ethernet = ETHERNET_ERR_BAD_PACKET;
            }

            if (pktDesc->pktLength > devicePtr->pktMTU) {
                error_code_ethernet = ETHERNET_ERR_BAD_PACKET;
            }

            // Count the number of frags in this packet
            fragcnt = 1;
            pktlen = pktDesc->pktLength;

            pktLastPtr = pktDesc;
            while (0U == (pktLastPtr->flags & ETHERNET_PKT_FLAG_EOP)) {
                if (NULL == pktLastPtr->nextPacketDesc) {
                    error_code_ethernet = ETHERNET_ERR_INVALID_PARAM;
                    break;
                }
                pktlen -= pktLastPtr->bufferLength;
                pktLastPtr = pktLastPtr->nextPacketDesc;
                fragcnt++;

                // At this point, we can't have another SOP
                if (0U != (pktLastPtr->flags & ETHERNET_PKT_FLAG_SOP)) {
                    error_code_ethernet = ETHERNET_ERR_BAD_PACKET;
                    break;
                }
            }

            // The final packet frag must be the last in the list
            if (NULL != pktLastPtr->nextPacketDesc) {
                error_code_ethernet = ETHERNET_ERR_BAD_PACKET;
            }

            // The frag count must be correct
            if (fragcnt != pktDesc->numPktFrags) {
                error_code_ethernet = ETHERNET_ERR_BAD_PACKET;
            }
        }

        if (ETHERNET_RET_SUCCESS == error_code_ethernet) {
            dmaObj = &devicePtr->dmaObj;
            txChan = &devicePtr->initConfig.chInfo[ETHERNET_CH_DIR_TX][pktDesc->pktChannel];

            // Get a local pointer to the descriptor channel
            ASSERT(NULL != dmaObj);
            channelDescPtr = &(dmaObj->txDma[pktDesc->pktChannel]);
            ASSERT(NULL != channelDescPtr);
            channelDescPtr->chInfo = txChan;

            // Make sure this packet does not have too many frags to fit
            if (fragcnt > channelDescPtr->descMax) {
                error_code_ethernet = ETHERNET_ERR_BAD_PACKET;
            }

            //Queue a packet and service transmitter

            if (ETHERNET_RET_SUCCESS == error_code_ethernet) {
                Ethernet_performPushPacketQueueChain(&channelDescPtr->waitQueue, pktDesc,
                    pktLastPtr, fragcnt);
                Ethernet_addPacketsIntoTxQueue(channelDescPtr);
            }
        }

        return error_code_ethernet;

*/







    }
}


void main(void)
{
    Ethernet_InitInterfaceConfig initInterfaceConfig;
    Ethernet_InitConfig *pInitCfg;

    //
    //Select the MII interface of the module
    //
    initInterfaceConfig.ssbase = EMAC_SS_BASE;
    initInterfaceConfig.enet_base = EMAC_BASE;
    initInterfaceConfig.phyMode = ETHERNET_SS_PHY_INTF_SEL_MII;
    //
    //Assign SoC specific functions for Enabling,Disabling interrupts
    //and for enabling the Peripheral at system level
    //
    initInterfaceConfig.ptrPlatformInterruptDisable = &Platform_disableInterrupt;
    initInterfaceConfig.ptrPlatformInterruptEnable = &Platform_enableInterrupt;
    initInterfaceConfig.ptrPlatformPeripheralEnable = &Platform_enablePeripheral;
    initInterfaceConfig.ptrPlatformPeripheralReset = &Platform_resetPeripheral;
    //
    //Assign the peripheral number at the SoC
    //
    initInterfaceConfig.peripheralNum = SYSCTL_PERIPH_CLK_ENET;
    //
    //Assign the default SoC specific interrupt numbers of Ethernet interrupts
    //
    initInterfaceConfig.interruptNum[0] = INT_EMAC;
    initInterfaceConfig.interruptNum[1] = INT_EMAC_TX0;
    initInterfaceConfig.interruptNum[2] = INT_EMAC_TX1;
    initInterfaceConfig.interruptNum[3] = INT_EMAC_RX0;
    initInterfaceConfig.interruptNum[4] = INT_EMAC_RX1;

    pInitCfg = Ethernet_initInterface(initInterfaceConfig);

    //
    // Get an initial configuration of known good parameters
    //
    Ethernet_getInitConfig(pInitCfg);

    //
    //Configure the Loopback mode
    //
    pInitCfg->loopbackMode = ETHERNET_MAC_CONFIGURATION_LM_LOOPBACK_DISABLED;
    
    //
    //Assign the callbacks for Getting packet buffer when needed
    //Releasing the TxPacketBuffer on Transmit interrupt callbacks
    //Receive packet callback on Receive packet completion interrupt
    //
    pInitCfg->pfcbGetPacket = &Ethernet_getPacketBuffer;
    pInitCfg->pfcbFreePacket = &Ethernet_releaseTxPacketBuffer;
    pInitCfg->pfcbRxPacket = &Ethernet_receivePacketCallback;
    //
    //Assign the Buffer to be used by the Low level driver for receiving
    //Packets. This should be accessible by the Ethernet DMA
    //
    pInitCfg->rxBuffer = Ethernet_rxBuffer;

    //
    //The Application handle is not used by this application
    //Hence using a dummy value of 1
    //
    Ethernet_getHandle((Ethernet_Handle)1,pInitCfg , &emac_handle);
    //
    //Do global Interrupt Enable
    //
    (void)Interrupt_enableInProcessor();
    //
    //Assign default ISRs
    //
    Interrupt_registerHandler(INT_EMAC_TX0, Ethernet_transmitISR);
    Interrupt_registerHandler(INT_EMAC_RX0, Ethernet_receiveISR);
    //
    //Enable the default interrupt handlers
    //
    Interrupt_enable(INT_EMAC_TX0);
    Interrupt_enable(INT_EMAC_RX0);

    //
    //Prepare a Packet Descriptor structure to send a packet
    //This contains a single buffer packet
    //The Source address shall be inserted by the MAC
    //Packet CRC is auto computed by the module and appended in the packet
    //

    pktDesc.dataOffset = 0;
    pktDesc.nextPacketDesc = 0;
    pktDesc.flags = ETHERNET_PKT_FLAG_SOP |ETHERNET_PKT_FLAG_EOP|ETHERNET_PKT_FLAG_SA_INS;
    pktDesc.pktChannel = ETHERNET_DMA_CHANNEL_NUM_0;

    pktDesc.numPktFrags = 1;

    //
    // Clear any IPC flags if set already
    //
    IPC_clearFlagLtoR(IPC_CM_L_CPU1_R, IPC_FLAG_ALL);

    //
    // Enable IPC interrupts
    //
    IPC_registerInterrupt(IPC_CM_L_CPU1_R, IPC_INT0, IPC_ISR0);

    //
    // Synchronize both the cores.
    //
    IPC_sync(IPC_CM_L_CPU1_R, IPC_FLAG31);

    //

    while (1) {
          //
          // Wait for the packet to be received
          //
    //  while (Ethernet_rxInterruptCount == 0);

          //
          // The control data to C28x side can be picked up from the Ethernet buffer and
          // passed to Ethernet
          //

          // Send response to C28x core
          if (status) {
              IPC_sendResponse(IPC_CM_L_CPU1_R, TEST_PASS);
          } else {
              IPC_sendResponse(IPC_CM_L_CPU1_R, TEST_FAIL);
          }

          //
          // Acknowledge the flag
          //
      //    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG0);
 //       IPC_Instance[IPC_CM_L_CPU1_R].IPC_Flag_Ctr_Reg->IPC_ACK = IPC_FLAG0;

        //read the status flag to see if acknowledgment is set and flag cleared in remote core


        int ackcounter=0;
       bool flag_ack_fail= false;
       int j;
       bool flag01=false;


       if(( IPC_Instance[IPC_CM_L_CPU1_R].IPC_Flag_Ctr_Reg->IPC_STS & IPC_FLAG0 != 0 )){

           flag01=true;

       }

        while((flag01) && (ackcounter <= 10) )
        {


            for(j=0 ; j<20;j++){

            }

            IPC_Instance[IPC_CM_L_CPU1_R].IPC_Flag_Ctr_Reg->IPC_ACK = IPC_FLAG0;

            ackcounter ++;


          // Reset the receive interrupt count for the next packet
        //  Ethernet_rxInterruptCount = 0;
        }



        if(( IPC_Instance[IPC_CM_L_CPU1_R].IPC_Flag_Ctr_Reg->IPC_STS & IPC_FLAG0 != 0 ))

                   flag_ack_fail=true;

               }
  }


//
// End of File
//
