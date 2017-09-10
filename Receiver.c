/*
 * Filename: Reciever.c
 *
 * Description: This is the firmware for the receiver to the TBI bioinstrument.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include<stdio.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>


/* Internal Clock*/
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/knl/Clock.h>

#include "hci.h"
/* Internal Clock*/

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/UART.h>

#include "bcomdef.h"

#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simple_gatt_profile.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "board_key.h"
//#include <ti/mw/display/Display.h>
#include "board.h"

#include "simple_central.h"

#include "ble_user_config.h"

#include <xdc/runtime/Log.h>   // For Log_warning1("Warning number #%d", 4); things
#include <xdc/runtime/Diags.h>

/* Internal Clock */
// This not employed due to Linker problem man
#define USE_RCOSC
#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC
/* Internal Clock*/
  char hello[20];
  char start[] = "\r\nStarting...";
UART_Handle uart;
UART_Params uartParams;
char uartTxBuffer[250];
void uartCallback(UART_Handle handle, void *buf, size_t count) {
   return;
}

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT               0x0001
#define SBC_PAIRING_STATE_EVT                 0x0002
#define SBC_PASSCODE_NEEDED_EVT               0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                    0x0010
#define SBC_STATE_CHANGE_EVT                  0x0020
#define SBC_MEASURE_SPEED_EVT                 0x0040

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          TRUE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               TRUE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400//400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                  1728
#endif

#define DLE_MAX_PDU_SIZE 251
#define DLE_MAX_TX_TIME 2120

//#define DEFAULT_PDU_SIZE 27
//#define DEFAULT_TX_TIME 328

#define DEFAULT_PDU_SIZE 251
#define DEFAULT_TX_TIME 2120

int reduction =0;
// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct 
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data 
} sbcEvt_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */

//Display_Handle dispHandle = NULL;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;
static Clock_Struct speedClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8_t scanRes;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Connection handle of current connection 
static uint16_t connHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Maximum PDU size (default = 27 octets)
static uint16 maxPduSize;

// Hard code the peer address
static uint8_t throughput_peripheral_peer_addr[B_ADDR_LEN] = { 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };

// Received byte counters
static volatile uint32_t bytesRecvd = 0;
static volatile uint32_t bytesRecvdShadow = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                         uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);

static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state, 
                                         uint8_t status);

void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keys);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status,
                                           uint8_t *pData);
static void SimpleBLECentral_speedHandler(UArg a0);

/* Internal Clock*/

#define RCOSC_CALIBRATION_PERIOD              1000

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      RCOSC_enableCalibration
 *
 * @brief   enable calibration.  calibration timer will start immediately.
 *
 * @param   none
 *
 * @return  none
 */
extern void RCOSC_enableCalibration(void);

// Clock instance for Calibration injections
static Clock_Struct  injectCalibrationClock;

// Power Notify Object for wake-up callbacks
Power_NotifyObj injectCalibrationPowerNotifyObj;

static uint8_t isEnabled = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void rcosc_injectCalibrationClockHandler(UArg arg);

static uint8_t rcosc_injectCalibrationPostNotify(uint8_t eventType,
                                                 uint32_t *eventArg,
                                                 uint32_t *clientArg);

/* Internal Clock*/

/*********************************************************************
 *
 *
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  (pfnPasscodeCB_t)SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;

  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);

  Util_constructClock(&speedClock, SimpleBLECentral_speedHandler,1000, 1000, false, 0);
  Board_initKeys(SimpleBLECentral_keyChangeHandler);

  //dispHandle = Display_open(Display_Type_LCD, NULL);

 // if(dispHandle == NULL)
  //{
   // dispHandle = Display_open(Display_Type_UART, NULL);

    //Send the form feed char to the LCD, this is helpful if using a terminal
    //as it will clear the terminal history
   // Display_print0(dispHandle, 0, 0, "\f");
  //}

  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;

    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t), 
                                &scanRes);
  }

  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, 
                   (void *)attDeviceName);

  GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, 6); //HEre is connection interval
  GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, 6); //HEre is connection interval
  GAP_SetParamValue(TGAP_CONN_EST_LATENCY, 10);
  GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, 200);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), 
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Add throughput_peripheral to whitelist
  uint8 bdAddressWhl[B_ADDR_LEN] = { 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
  // Add peer 0xAABBCCDDEEFF to white list.
  HCI_LE_AddWhiteListCmd(HCI_PUBLIC_DEVICE_ADDRESS, bdAddressWhl);

  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleBLECentral_bondCB);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

 // Display_print0(dispHandle, 0, 0, "Throughput Central_No_CLock_max_PDU");

  /* Internal Clock*/
//#ifdef USE_RCOSC
RCOSC_enableCalibration();
//#endif // USE_RCOSC
/* Internal Clock*/

Board_initUART();

    /* Create a UART with data processing off. */
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.writeMode = UART_MODE_BLOCKING; // that's a problem here, you need to block the sampling task
       // uartParams.writeCallback = uartCallback;
        uartParams.baudRate = 115200;
        uart = UART_open(Board_UART0, &uartParams);

        UART_write(uart, start, sizeof(start));
        System_printf("UART did not open");
        System_flush();

}

/*********************************************************************
 * @fn      SimpleBLECentral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLECentral_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest, 
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    while (!Queue_empty(appMsgQueue))
    {
      sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLECentral_processAppMsg(pMsg);

        // Free the space from the message
        ICall_free(pMsg);
      }
    }

    if (events & SBC_START_DISCOVERY_EVT) // Not bit-bit operator -> 011 & 011 = 1
    {
      events &= ~SBC_START_DISCOVERY_EVT; // set it back to zero 0

      SimpleBLECentral_startDiscovery();
    }
    if (events & SBC_MEASURE_SPEED_EVT)
    {
      events &= ~SBC_MEASURE_SPEED_EVT;
     uint_fast16_t ss=0;
     ss = System_sprintf(hello, "\r\n Data rate :%d",bytesRecvdShadow );
     UART_write(uart, hello, ss);



      //Display_print1(dispHandle, 7, 0, "\t\t\t\tRate (B/s): %d", bytesRecvdShadow);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;

    case GATT_MSG_EVENT:
      SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            SimpleBLECentral_processCmdCompleteEvt((hciEvt_CmdComplete_t *)pMsg);
            break;

          default:
            break;
        }
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);

      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;

    case SBC_KEY_CHANGE_EVT:
      SimpleBLECentral_handleKeys(0, pMsg->hdr.state); 
      break;

    // Pairing event
    case SBC_PAIRING_STATE_EVT:
      {
        SimpleBLECentral_processPairState(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    // Passcode event
    case SBC_PASSCODE_NEEDED_EVT:
      {
        SimpleBLECentral_processPasscode(connHandle, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;
      }

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT: //Consider connecting here
      {
        maxPduSize = pEvent->initDone.dataPktLen;

      //  Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
      //  Display_print0(dispHandle, 2, 0, "Initialized");
       // Display_print0(dispHandle, 2, 0, "Press left and right keys to connect to throughput_peripheral");
       while( !GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,DEFAULT_LINK_WHITE_LIST,ADDRTYPE_PUBLIC, throughput_peripheral_peer_addr))
       {}


      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (SimpleBLECentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                           pEvent->deviceInfo.pEvtData,
                                           pEvent->deviceInfo.dataLen))
          {
            SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr, 
                                           pEvent->deviceInfo.addrType);
          }
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {

        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          scanRes = pEvent->discCmpl.numDevs;
          memcpy(devList, pEvent->discCmpl.pDevList,
                 (sizeof(gapDevRec_t) * scanRes));
        }

     //   Display_print1(dispHandle, 2, 0, "Devices Found %d", scanRes);

        if (scanRes > 0)
        {
    //      Display_print0(dispHandle, 3, 0, "<- To Select");
        }

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        attExchangeMTUReq_t req;
        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle = pEvent->linkCmpl.connectionHandle;

          // If service discovery not performed initiate service discovery
          if (charHdl == 0)
          {
            Util_startClock(&startDiscClock);
          }
          Util_startClock(&speedClock);

        //  Display_print0(dispHandle, 2, 0, "Connected");
        //  Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr));



          // Discover GATT Server's Rx MTU size
        req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;
      //  Display_print1(dispHandle, 2, 0, "DISCOVER %d",req.clientRxMTU);


          VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);


             return;
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle = GAP_CONNHANDLE_INIT;
          discState = BLE_DISC_STATE_IDLE;

        //  Display_print0(dispHandle, 2, 0, "Connect Failed");
         // Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->gap.hdr.status);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle = GAP_CONNHANDLE_INIT;
        discState = BLE_DISC_STATE_IDLE;
        charHdl = 0;

    //    Display_print0(dispHandle, 2, 0, "Disconnected");
    //    Display_print1(dispHandle, 3, 0, "Reason: %d", pEvent->linkTerminate.reason);
     //   Display_clearLine(dispHandle, 4);
        while( !GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,DEFAULT_LINK_WHITE_LIST,ADDRTYPE_PUBLIC, throughput_peripheral_peer_addr))
              {}
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
   //     Display_print1(dispHandle, 2, 0, "Param Update: %d", pEvent->linkUpdate.status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  // Check for both keys first
  //if (keys == (KEY_LEFT | KEY_RIGHT))
  //{
    // Immediately assume that throughput_peripheral is advertising and connect
    GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                 DEFAULT_LINK_WHITE_LIST,
                                 ADDRTYPE_PUBLIC, throughput_peripheral_peer_addr);

    return;
  //}

 // if (keys & KEY_LEFT)
 // {
  //  return;
 // }

 // if (keys & KEY_RIGHT)
 // {

  //  return;
 // }

}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg) // LE Where the action happen
{
  if (state == BLE_STATE_CONNECTED)
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
     // Display_print1(dispHandle, 4, 0, "ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
      //  Display_print1(dispHandle, 4, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
    //    Display_print1(dispHandle, 4, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }

    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
    //    Display_print1(dispHandle, 4, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and
        // increment value
    //    Display_print1(dispHandle, 4, 0, "Write sent: %d", charVal++);
      }

    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
    //  Display_print1(dispHandle, 4, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
	}
    else if (pMsg->method == ATT_HANDLE_VALUE_NOTI) // LE
    {
     //   char *a=(char*)malloc(sizeof(char)*4);
       // char a [10];
      bytesRecvd += pMsg->msg.handleValueNoti.len;
     // Display_print0(dispHandle, 7, 0, "---------------------------------------");   // Added .. This crap automatically adds a newline... will be parsed later one or use system_printf
     // Display_print1(dispHandle, 7, 0, "Byte0: %d", pMsg->msg.readRsp.pValue[0]); // Added
     /// Display_print1(dispHandle, 7, 1, "Byte1: %d", pMsg->msg.readRsp.pValue[1]); // Added
     // Display_print1(dispHandle, 7, 2, "Byte2: %d", pMsg->msg.readRsp.pValue[2]); // Added
     // Display_print1(dispHandle, 7, 3, "Byte3: %d\n", pMsg->msg.readRsp.pValue[3]); // Added
     // sprintf(a,"%02x %02x %02x %02x %02x %02x %02x %02x %02x",pMsg->msg.readRsp.pValue[0],pMsg->msg.readRsp.pValue[1],pMsg->msg.readRsp.pValue[2],pMsg->msg.readRsp.pValue[3],pMsg->msg.readRsp.pValue[0],pMsg->msg.readRsp.pValue[1],pMsg->msg.readRsp.pValue[2],pMsg->msg.readRsp.pValue[3],pMsg->msg.readRsp.pValue[0]);

     // for (int i=0;i<16;i++)
     // {
     //     sprintf(a,"%02x ", pMsg->msg.readRsp.pValue[i]);

    //  }

      uint_fast16_t i;
      uint_fast16_t uartTxBufferOffset=0;

         /*Momen*/
 //reduction = reduction+1;
//if(reduction>3){



          UART_write(uart, "\r\n",2);
          uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"\r\n");

          for(i=0; i <pMsg->msg.handleValueNoti.len-2; i=i+2)
           {

         uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"%01x",pMsg->msg.readRsp.pValue[i]);
         uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"%02x",pMsg->msg.readRsp.pValue[i+1]);
        // uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"%03x",pMsg->msg.readRsp.pValue[i]<<8 + pMsg->msg.readRsp.pValue[i+1]);

            }
         uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"\r\n%d",pMsg->msg.readRsp.pValue[224]);


           // UART_write(uart, "123,",255);
           UART_write(uart, uartTxBuffer, uartTxBufferOffset);
//reduction = 0;
//}

/*Momen2

      //UART_write(uart, "\r\n",2);
      uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"\r\n");

      for(i=0; i <pMsg->msg.handleValueNoti.len; i=i++)
       {

      uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"%02x",pMsg->msg.readRsp.pValue[i]);

        }
      uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"\r\n%d",pMsg->msg.readRsp.pValue[pMsg->msg.handleValueNoti.len-1]);



       // UART_write(uart, "123,",255);
       UART_write(uart, uartTxBuffer, uartTxBufferOffset);
//reduction = 0;
//}

*/


/*
         // PETKOS

                  UART_write(uart, "\r\n",3);
                   for(i=0; i <pMsg->msg.handleValueNoti.len; i=i+2)
                   {

                  uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"\r\n%02x",pMsg->msg.readRsp.pValue[i]);
                  uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"%02x",pMsg->msg.readRsp.pValue[i+1]);
                     }
                 uartTxBufferOffset += System_sprintf(uartTxBuffer + uartTxBufferOffset,"%d",pMsg->msg.readRsp.pValue[224]);
                        //   uartTxBuffer[uartTxBufferOffset] = '\n';


                   // UART_write(uart, "123,",255);
                   UART_write(uart, uartTxBuffer, uartTxBufferOffset);


*/




    //  sprintf(a,"%01x%02x %01x%02x %01x%02x %01x%02x %01x%02x %01x%02x %01x%02x",pMsg->msg.readRsp.pValue[0],pMsg->msg.readRsp.pValue[1],pMsg->msg.readRsp.pValue[2],pMsg->msg.readRsp.pValue[3],pMsg->msg.readRsp.pValue[4],pMsg->msg.readRsp.pValue[5],pMsg->msg.readRsp.pValue[6],pMsg->msg.readRsp.pValue[7],pMsg->msg.readRsp.pValue[8],pMsg->msg.readRsp.pValue[9],pMsg->msg.readRsp.pValue[10],pMsg->msg.readRsp.pValue[11],pMsg->msg.readRsp.pValue[12],pMsg->msg.readRsp.pValue[13]);
   //   Display_print1(dispHandle, 7, 0, "%s", a); // Added each PETKOS
     // Display_print0(dispHandle, 0, 0, "[]");
     // System_printf("Hello, universe!\r\n");
      //pMsg->msg.readRsp.pValue[3] = '\0';
//      char momen [50];
//      momen[0] = itoa(pMsg->msg.readRsp.pValue[0]);
//      momen[1] = pMsg->msg.readRsp.pValue[1];
    //Display_print1(dispHandle, 7, 0, " %s",a);
   //   free (a);
//      Display_print1(dispHandle, 7, 0, " %s", momen);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
   //   Display_print1(dispHandle, 4, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SimpleBLECentral_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
      {
        int8 rssi = (int8)pMsg->pReturnParam[3];

      //  Display_print1(dispHandle, 4, 0, "RSSI -dB: %d", (uint32_t)(-rssi));
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
   // Display_print0(dispHandle, 2, 0, "Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
   //   Display_print0(dispHandle, 2, 0, "Pairing success");
    }
    else
    {
   //   Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
  //    Display_print0(dispHandle, 2, 0, "Bonding success");
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
  //    Display_print0(dispHandle, 2, 0, "Bond save success");
    }
    else
    {
   //   Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
 //   Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleBLECentral_startDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = charHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;


  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple BLE service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

      discState = BLE_DISC_STATE_SVC;

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle, uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) && 
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_ReadUsingCharUUID(connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) && 
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                             pMsg->msg.readByTypeRsp.pDataList[1]);

   //   Display_print0(dispHandle, 2, 0, "Simple Svc Found");
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData, 
                                         uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) || 
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;

        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }

          // Go to next
          pData += 2;
          adLen -= 2;
        }

        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;

    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT, 
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }

  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = status;

    // Queue the event.
    SimpleBLECentral_enqueueMsg(SBC_PAIRING_STATE_EVT, state, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  uint8_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = uiOutputs;

    // Enqueue the event.
    SimpleBLECentral_enqueueMsg(SBC_PASSCODE_NEEDED_EVT, 0, pData);
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
  events |= SBC_START_DISCOVERY_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8 keys)
{
  SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t state, 
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }

  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_speedHandler
 *
 * @brief   RTOS clock handler that counts number of bytes recieved
 *			in a clock period.
 *
 * @param   a0 - RTOS clock arg0.
 *
 * @return  void
 */
static void SimpleBLECentral_speedHandler(UArg a0)
{
  bytesRecvdShadow = bytesRecvd;
  bytesRecvd = 0;
  events |= SBC_MEASURE_SPEED_EVT;
  Semaphore_post(sem);
}

/* Internal Clock*/
/*********************************************************************
*********************************************************************/
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      RCOSC_enableCalibration
 *
 * @brief   enable calibration.  calibration timer will start immediately.
 *
 * @param   none
 *
 * @return  none
 */
void RCOSC_enableCalibration(void)
{
  if (!isEnabled)
  {
    isEnabled = TRUE;

    // Set device's Sleep Clock Accuracy
    HCI_EXT_SetSCACmd(500);

    // Create RCOSC clock - one-shot clock for calibration injections.
    Util_constructClock(&injectCalibrationClock,
                        rcosc_injectCalibrationClockHandler,
                        RCOSC_CALIBRATION_PERIOD, 0, false, 0);

    // Receive callback when device wakes up from Standby Mode.
    Power_registerNotify(&injectCalibrationPowerNotifyObj, PowerCC26XX_AWAKE_STANDBY,
                         (Power_NotifyFxn)rcosc_injectCalibrationPostNotify,
                         NULL);


    // Start clock for the RCOSC calibration injection.  Calibration must be
    // done once every second by either the clock or by waking up from StandyBy
    // Mode. To ensure that the device is always correctly calibrated the clock
    // is started now but is only allowed to expire when a wake up event does
    // not occur within the clock's duration.
    Util_startClock(&injectCalibrationClock);
  }
}

/*********************************************************************
 * @fn      rcosc_injectCalibrationClockHandler
 *
 * @brief   Handler function for RCOSC clock timeouts.  Executes in
 *          SWI context.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void rcosc_injectCalibrationClockHandler(UArg arg)
{
  // Restart clock.
  Util_startClock(&injectCalibrationClock);

  // Inject calibration.
  PowerCC26XX_injectCalibration();
}

/*********************************************************************
 * @fn      rcosc_injectCalibrationPostNotify
 *
 * @brief   Callback for Power module state change events.
 *
 * @param   eventType - The state change.
 * @param   clientArg - Not used.
 *
 * @return  Power_NOTIFYDONE
 */
static uint8_t rcosc_injectCalibrationPostNotify(uint8_t eventType,
                                                 uint32_t *eventArg,
                                                 uint32_t *clientArg)
{
  // If clock is active at time of wake up,
  if (Util_isActive(&injectCalibrationClock))
  {
    // Stop injection of calibration - the wakeup has automatically done this.
    Util_stopClock(&injectCalibrationClock);
  }

  // Restart the clock in case delta between now and next wake up is greater
  // than one second.
  Util_startClock(&injectCalibrationClock);

  return Power_NOTIFYDONE;
}
//#endif //USE_RCOSC

/* Internal Clock*/
