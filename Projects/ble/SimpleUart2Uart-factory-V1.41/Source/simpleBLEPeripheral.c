/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "simpleble.h"
#include "npi.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "amomcu_buffer.h"
#include "VibrativeSensor.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6//80   连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     6//800   连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          100//1000  -各种原因断开连接后，超时并重新广播的时间:  100 = 1s

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         3

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
//uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

gaprole_States_t gapProfileState = GAPROLE_INIT;

bool simpleBLEChar6DoWrite2 = TRUE;
bool simpleBLEChar7DoWrite = TRUE;
bool timerIsOn = FALSE;          // 

// Connection handle
uint16 gapConnHandle = NULL;

/*
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};
*/

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData_iBeacon[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // in this peripheral
  0x1A,   // length of this data 26byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,  
  /*Apple Pre-Amble*/
  0x4C,
  0x00,
  0x02,
  0x15,
  /*Device UUID (16 Bytes)*/
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48,0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0,
  /*Major Value (2 Bytes)*/
  0x00, 0x01,
  
  /*Minor Value (2 Bytes)*/
  0x00,0x02,
  
  /*Measured Power*/
  0xCD
};
//static int8 gMP = 0xCD;
// GAP GATT Attributes

// GAP GATT Attributes
//static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void peripheralRssiReadCB( int8 rssi );
static void simpleProfileChangeCB( uint8 paramID );

//#if defined( BLE_BOND_PAIR )
typedef enum
{
  BOND_PAIR_STATUS_PAIRING,  //未配对
  BOND_PAIR_STATUS_PAIRED,  //已配对
}BOND_PAIR_STATUS;
// 用来管理当前的状态，如果密码不正确，立即取消连接，并重启
static BOND_PAIR_STATUS gPairStatus = BOND_PAIR_STATUS_PAIRING;

void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs );
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status );
//#endif

//#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
//#endif




/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  peripheralRssiReadCB,               // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
//#if defined( BLE_BOND_PAIR )
  ProcessPasscodeCB,                     // 密码回调
  ProcessPairStateCB                     // 绑定状态回调
//#else
//  NULL,                     // Passcode callback (not used by application)
//  NULL                      // Pairing / Bonding state Callback (not used by application)
//#endif
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
#if 0
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
#else
    {       
        uint8 AttDeviceNameLen = osal_strlen((char*)simpleBle_GetAttDeviceName());
        uint8 pSscanRspDataLen = ( 11 + AttDeviceNameLen);
        uint8 *pSscanRspData = osal_mem_alloc(pSscanRspDataLen);
        if(pSscanRspData)
        {
            uint8 index = 0;
            
            pSscanRspData[0] = AttDeviceNameLen + 1;
            pSscanRspData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
            osal_memcpy(&pSscanRspData[2], simpleBle_GetAttDeviceName(), AttDeviceNameLen);

            index = 2 + AttDeviceNameLen;
            
            pSscanRspData[index+0] = 0x05;
            pSscanRspData[index+1] = GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE;
            pSscanRspData[index+2] = LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL );   // 100ms
            pSscanRspData[index+3] = HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL );
            pSscanRspData[index+4] = LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL );   // 1s
            pSscanRspData[index+5] = HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL );

            // Tx power level
            pSscanRspData[index+6] = 0x02;   // length of this data
            pSscanRspData[index+7] = GAP_ADTYPE_POWER_LEVEL;
            pSscanRspData[index+8] = 0;       // 0dBm    

            GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, pSscanRspDataLen, pSscanRspData );

            osal_mem_free(pSscanRspData);
        }
        else
        {
            HalLcdWriteString( "ERR:pSscanRspData", HAL_LCD_LINE_1 );
        }
    }
#endif

    if(simpleBLE_CheckIfUse_iBeacon())
    {
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData_iBeacon ), advertData_iBeacon );
    }
    else
    {
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    }

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  //GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, simpleBle_GetAttDeviceName() );

  {
     // 设置rssi 参数更新速率
     uint16 rssi_read_rate_1ms = 500; //一秒更新2次   
     GAPRole_SetParameter(GAPROLE_RSSI_READ_RATE, sizeof( uint16 ), &rssi_read_rate_1ms);
  }
    
  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    if(simpleBLE_CheckIfUse_iBeacon())
    {
        advInt = simpleBLE_GetiBeaconAdvertisingInterral();
        advInt = (advInt*1000 / 625);
    }
    
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  //下面是与配对相关的设置  
  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;  //显示密码， 以便主机输入配对的密码

    /*
    bonding就是把配对信息记录下来, 下次就不用配对了. 不bonding下次就还得配对.    
    所以我们从机这里把 bonding = FALSE 的后果就是， 主设备每次连接都必须输入密码
    而把  bonding = TRUE 后， 主设备只需第一次连接时输入密码， 后面断开后都不需要再次输入密码即可连接
    ---------------amomcu.com-------------------------    
    */
    uint8 bonding = FALSE;   

//#if defined( BLE_BOND_PAIR )
    if(simpleBle_GetIfNeedPassword())
    {
        pairMode = GAPBOND_PAIRING_MODE_INITIATE;   //配对模式，置配成等待主机的配对请求
    }
    else
    {
        pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    }    
//#endif

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
//    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 1, 2, 3, 4, 5 };
//    uint8 charValue7[SIMPLEPROFILE_CHAR7_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, charValue7 );
  }


#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low
#else
  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
#endif // #if defined( CC2540_MINIDK )


#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  // 需要关闭的CLK自动分频，在初始化中加入HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT )?  // 如果开启，会导致频率自动切换，DMA工作受到影响，小范围丢数。  
  // 这里把他关闭， 如果想降低功耗， 这个应该要开启的， 这里矛盾了  
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );
  //HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // 信号发射强度  
  HCI_EXT_SetTxPowerCmd(3 - sys_config.txPower);


  HalLcdWriteString("   WWW.AMOMCU.COM", HAL_LCD_LINE_8);
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {    
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_LED_EVT, 100);

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

#if defined ( AMOMCU_UART_RX_MODE)
    osal_start_timerEx( simpleBLETaskId, SBP_SLEEP_EVT, SLEEP_MS);
#endif

    CheckKeyForSetAllParaDefault(); //按键按下3秒， 恢复出厂设置
    
    // 延时400ms后唤醒， 不然会继续睡眠，原因不明
    osal_start_timerEx( simpleBLETaskId, SBP_WAKE_EVT, 500 );   
    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & SBP_WAKE_EVT )
  {      
    g_sleepFlag = FALSE;
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  不睡眠，功耗很高的        
    return ( events ^ SBP_WAKE_EVT );
  }

  if ( events & SBP_DATA_EVT )
  {
#if 1   
    uint8 send_count = 3;
    while(send_count--)
    {
        uint16 totalbytes = qq_total();
        uint8 numBytes;    

//        totalbytes = 20;
        
          // 读  RdLen 个字节数据到 缓冲区 RdBuf， 返回读取到的有效数据长度
        if(totalbytes > 0 &&  simpleBLEChar6DoWrite2)
        {            
            numBytes = ( (totalbytes > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : totalbytes);

            if(simpleBLEChar6DoWrite2)               //写入成功后再写入
            {     
    //            uint8 buf[SIMPLEPROFILE_CHAR6_LEN];

#if 0 // 这种速度慢 SimpleProfile_SetParameter           
    //            simpleBLEChar6DoWrite2 = FALSE;
                qq_read(buf, numBytes);
                SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, numBytes, buf );
#else // 这种速度快 GATT_Notification
        // 目前有问题， 不要启用
                attHandleValueNoti_t pReport;
                pReport.pValue = GATT_bm_alloc( gapConnHandle, ATT_WRITE_REQ, numBytes, NULL );
                if(pReport.pValue != NULL)
                {
                    pReport.len = numBytes;
                    pReport.handle = 0x0035;
                    
                    qq_read(pReport.pValue, pReport.len);
                    
                    GATT_Notification(gapConnHandle, &pReport, FALSE );            
                }
//                else
//                {
//                    LCD_WRITE_STRING_VALUE( "err: line=", __LINE__, 10, HAL_LCD_LINE_1 );
//                    HalLedSet(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, HAL_LED_MODE_ON);
//                    while(1);
//                }
#endif      
            }
            else
            {
                LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
            }   
          }
          else
          {
                break;
          }
      } 
    
      if(qq_total() > 0)
      {
         timerIsOn = TRUE;
         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 6);
//         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 16);
      }
      else
      {
         timerIsOn = FALSE;
      }
#else

#endif
      return ( events ^ SBP_DATA_EVT );
  }
  
#if 0  
  if ( events & SBP_VABRATIVE_EVT)
  { 
//    uint8 send_count = 3;
//    while(send_count--)
//    {
//        uint16 totalbytes = qq_total();
//        uint8 numBytes;    
//
////        totalbytes = 20;
//        
//          // 读  RdLen 个字节数据到 缓冲区 RdBuf， 返回读取到的有效数据长度
//        if(totalbytes > 0 &&  simpleBLEChar7DoWrite)
//        {            
//
            if(simpleBLEChar7DoWrite)               //写入成功后再写入
            {     
                attHandleValueNoti_t pReport;
                pReport.pValue = GATT_bm_alloc( gapConnHandle, ATT_WRITE_REQ, 1, NULL );
//                if(pReport.pValue != NULL)
//                {
                    pReport.len = 1;
                    pReport.handle = 0x0035;
                    
                    pReport.pValue=&vibra_value;//, pReport.len);
                    
                    GATT_Notification(gapConnHandle, &pReport, FALSE );            
//                }
            }
//            else
//            {
//                LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
//            }   
//          }
//          else
//          {
//                break;
//          }
//      } 
//    
//      if(qq_total() > 0)
//      {
//         timerIsOn = TRUE;
//         osal_start_timerEx( simpleBLETaskId, SBP_VABRATIVE_EVT, 6);
////         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 16);
//      }
//      else
//      {
//         timerIsOn = FALSE;
//      }
      return ( events ^ SBP_VABRATIVE_EVT );
  }
#endif

  if ( events & SBP_UART_EVT )
  {      
      if(FALSE == timerIsOn)
      {
//         osal_set_event( simpleBLETaskId, SBP_DATA_EVT );
         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 6);
      }      
      return ( events ^ SBP_UART_EVT );
  }  

#if defined ( AMOMCU_UART_RX_MODE)
  if ( events & SBP_SLEEP_EVT )
  {      
      extern uint8 uart_sleep_count; 

      if(uart_sleep_count > 3)
      {
        if(g_sleepFlag == FALSE)
        {
            char strTemp[64];

            g_sleepFlag = TRUE;

            //设置p02的功能，1为uart脚， 0为输入中断脚
            HalKey_Set_P02_for_UartRX_or_GPIO(false);
            osal_pwrmgr_device( PWRMGR_BATTERY);   //  自动睡眠
            //osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  不睡眠，功耗很高的        
            // 格式化
            sprintf(strTemp, "+SLEEP\r\nOK\r\n");
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            // 为了让串口数据发送完毕， 需要先延时一下，否则进入了睡眠就发送乱码了， 1ms即可
            simpleBLE_Delay_1ms(1);
        }
      }
      else
      {
          uart_sleep_count++;
      }

      osal_start_timerEx( simpleBLETaskId, SBP_SLEEP_EVT, SLEEP_MS);
      
      return ( events ^ SBP_SLEEP_EVT );
  }
#else
#endif

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    simpleBLE_performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

  if ( events & SBP_PERIODIC_LED_EVT )
  { // 开机测试led闪烁几次
    static uint8 led_conut = 0;

    if(++led_conut < 10)
    {
        HalLedSet(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, HAL_LED_MODE_TOGGLE);
        osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_LED_EVT, 100 );
    }
    else
    {
        HalLedSet(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, HAL_LED_MODE_OFF);
    }

    return (events ^ SBP_PERIODIC_LED_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
//  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
//  #endif // #if defined( CC2540_MINIDK )

    case GATT_MSG_EVENT:
      // Process GATT message
      simpleBLEPeripheral_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
    // do nothing
    break;
  }
}

//#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
     /*
    按键处理公共函数， 主机与从机都是运行这个函数，
    注意每次启动不是主机就是从机，不是同时是主机与从机的， 所以他们不冲突的
    */
    simpleBLE_HandleKeys(keys);
 }
//#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

        //mac 地址   bdAddr2Str( pEvent->initDone.devAddr ) 的输出格式是， 例如     0xD03972A5F3 
        osal_memcpy(sys_config.mac_addr, bdAddr2Str( ownAddress )+2, 12);
        sys_config.mac_addr[12] = 0;
        //PrintAllPara();
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        simpleBle_LedSetState(HAL_LED_MODE_OFF);
        NPI_WriteTransport("Connected\r\n", 11);
        simpleBLE_Delay_1ms(1);  //为了等发送完整所以延时一小下

        // Get connection handle
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);

        // 连接上了
        g_sleepFlag = FALSE;
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  不睡眠，功耗很高的        
      }
      break;

    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif

    //LCD_WRITE_STRING( "?", HAL_LCD_LINE_1 );

}


static void peripheralRssiReadCB( int8 rssi )
{
    simpleBle_SetRssi(rssi);
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint8 newChar6Value[SIMPLEPROFILE_CHAR6_LEN];
  uint8 returnBytes;
  
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue, &returnBytes );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue, &returnBytes );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;
    
    case SIMPLEPROFILE_CHAR6:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, newChar6Value, &returnBytes );
      if(returnBytes > 0)
      {
        if(simpleBLE_CheckIfUse_Uart2Uart())     //使用透传模式时才透传
        {
            NPI_WriteTransport(newChar6Value,returnBytes);
            
            // 这里可以处理一下数据，比如发命令点灯
            // MT 命令处理 函数
            simpleBLE_MT_CMD_Handle(newChar6Value,returnBytes);
          }
      }

      break;
      
    default:
      // should not reach here!
      break;
  }
}

//#if defined( BLE_BOND_PAIR )
//绑定过程中的密码管理回调函数
static void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs )
{
  uint32  passcode;
  uint8   str[7];

  //在这里可以设置存储，保存之前设定的密码，这样就可以动态修改配对密码了。
  // Create random passcode
#if 0
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;  
#else
  passcode = simpleBle_GetPassword();
#endif
  //在lcd上显示当前的密码，这样手机端，根据此密码连接。
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    HalLcdWriteString( "Passcode:",  HAL_LCD_LINE_1 );
    HalLcdWriteString( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }

  if(simpleBle_GetIfNeedPassword())
  {
    HalLcdWriteString( ">>>need Passcode",  HAL_LCD_LINE_1 );    
    
    //  串口输出密码
    simpleBle_PrintPassword();
    
    // Send passcode response  发送密码请求给主机
    GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
  }
  else
  {
    // 无需密码
    HalLcdWriteString( ">>>no Passcode",  HAL_LCD_LINE_1 );    
  }
}

//绑定过程中的状态管理，在这里可以设置标志位，当密码不正确时不允许连接。
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  // 主机发起连接，会进入开始绑定状态
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    HalLcdWriteString( "Pairing started", HAL_LCD_LINE_1 );
	gPairStatus = BOND_PAIR_STATUS_PAIRING;
  }
  // 当主机提交密码后，会进入完成
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      HalLcdWriteString( "Pairing success", HAL_LCD_LINE_1 );/*密码正确*/
	  gPairStatus = BOND_PAIR_STATUS_PAIRED;
    }
    else
    {
      HalLcdWriteStringValue( "Pairing fail", status, 10, HAL_LCD_LINE_1 );/*密码不正确，或者先前已经绑定*/
      
	  if(status ==8)
	  {//已绑定
		gPairStatus = BOND_PAIR_STATUS_PAIRED;
	  }
      else
      {
		gPairStatus = BOND_PAIR_STATUS_PAIRING;

        GAPRole_TerminateConnection();  // 终止连接
        simpleBLE_Delay_1ms(100);
        
        // 终止连接后， 需要复位从机
        HAL_SYSTEM_RESET();        
	  }
      
      gPairStatus = BOND_PAIR_STATUS_PAIRING;
    }
    
	//判断配对结果，如果不正确立刻停止连接。
	if((gapProfileState == GAPROLE_CONNECTED) && (gPairStatus == BOND_PAIR_STATUS_PAIRING))
    {
	  GAPRole_TerminateConnection();  // 终止连接
      // 终止连接后， 需要复位从机
      HAL_SYSTEM_RESET();
    }
  }
  // 当主机提交密码从机验证后进入配对成功状态
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      HalLcdWriteString( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}
//#endif
/*********************************************************************
*********************************************************************/
