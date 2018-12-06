/**************************************************************************************************
  Filename:       simpleBLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED ?S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"

#include "npi.h"
#include "simpleble.h"
#include "stdio.h"
#include "string.h"
#include "amomcu_buffer.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  6//8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1500//4000  默认扫描时间 ms

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      6//400  连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      6//800  连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
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

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

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

// Task ID for internal task/event processing
uint8 simpleBLETaskId;

// GAP GATT Attributes
//static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;  //扫描结果
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection 
uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
uint16 simpleBLECharHdl = 0;
uint16 simpleBLECharHd6 = 0;
bool simpleBLEChar6DoWrite = TRUE;
bool simpleBLECentralCanSend = FALSE;          //  主机可发送数据

extern bool timerIsOn;          // 


// Value to write
//static uint8 simpleBLECharVal = 0;

// Value read/write toggle
//static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
//static bool simpleBLEProcedureInProgress = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );  
  //GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, simpleBle_GetAttDeviceName() );


  // Setup the GAP Bond Manager
  {
    uint32 passkey = simpleBle_GetPassword();
    uint8 pairMode;// = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    //uint8 bonding = DEFAULT_BONDING_MODE;

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
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  HalLcdWriteString("   WWW.AMOMCU.COM", HAL_LCD_LINE_8);
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
}

void simpleBLEStartScan()
{
//     LCD_WRITE_STRING( "--simpleBLEStartScan", HAL_LCD_LINE_2 );
      
     simpleBLECentralCanSend = FALSE;
    
    //开机后就马上进行，自动开始搜索
    if ( !simpleBLEScanning/* & simpleBLEScanRes == 0 */)
    {
      simpleBLEScanning = TRUE;
      simpleBLEScanRes = 0;
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                     DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                     DEFAULT_DISCOVERY_WHITE_LIST );   
      LCD_WRITE_STRING( "Start Scanning...", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING( "In Scanning...", HAL_LCD_LINE_1 );
    }
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );
    
    //simpleBLEStartScan();

    osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    CheckKeyForSetAllParaDefault(); //按键按下3秒， 恢复出厂设置

    // 判断 如如果设置过 AT+IMME0 的， 则去连接最近连接过的从机 
    if(1 == sys_config.workMode)
    {
        g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_CONNL;

        // 开始扫描
        simpleBLEStartScan();
    }

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

  if ( events & SBP_CONNECT_EVT )
  {      
    attWriteReq_t AttReq;       
    AttReq.handle = (simpleBLECharHd6+1)/*0x0036*/;       
    AttReq.len = 2;
    AttReq.sig = 0;
    AttReq.cmd = 0;
    AttReq.pValue = GATT_bm_alloc( simpleBLEConnHandle, ATT_WRITE_REQ, 2, NULL );
    if (AttReq.pValue != NULL)
    {   
        AttReq.pValue[0] = 0x01;
        AttReq.pValue[1] = 0x00;        

//        LCD_WRITE_STRING_VALUE( "AttReq.handle=", AttReq.handle, 10, HAL_LCD_LINE_1 );

        GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
//        LCD_WRITE_STRING( "Enable Notice\n", HAL_LCD_LINE_1);           

        simpleBLECentralCanSend = TRUE;

        NPI_WriteTransport("Connected\r\n", 11); 
        LCD_WRITE_STRING("Connected", HAL_LCD_LINE_3);
    }
    else
    {
        LCD_WRITE_STRING_VALUE( "err: line=", __LINE__, 10, HAL_LCD_LINE_1 );
        while(1);
    }

    return ( events ^ SBP_CONNECT_EVT );
  }  

  if ( events & SBP_DATA_EVT )
  {
#if 1   
    uint16 totalbytes = qq_total();
    uint8 numBytes;
    
      // 读  RdLen 个字节数据到 缓冲区 RdBuf， 返回读取到的有效数据长度
    if(totalbytes > 0 &&  simpleBLEChar6DoWrite)
    {            
        attWriteReq_t AttReq;           
        numBytes = ( (totalbytes > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : totalbytes);
        
//        LCD_WRITE_STRING_VALUE( "s=", numBytes, 10, HAL_LCD_LINE_1 );        
        
        AttReq.pValue = GATT_bm_alloc( simpleBLEConnHandle, ATT_WRITE_REQ, numBytes, NULL );
        if ( AttReq.pValue != NULL )
        {   
            AttReq.handle = simpleBLECharHd6;
            AttReq.len = numBytes;
            AttReq.sig = 0;
            AttReq.cmd = 0;

            //osal_memcpy(AttReq.pValue,buf,AttReq.len);
            qq_read(AttReq.pValue, AttReq.len);

//            LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
            //GATT_WriteCharValue( simpleBLEConnHandle, &AttReq, simpleBLETaskId );
            GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );

            simpleBLEChar6DoWrite = FALSE;

//            LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
        }
        else
        {
//            LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
        }   
    }

      if(qq_total() > 0)
      {
         timerIsOn = TRUE;
//         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 1);
         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 16);
      }
      else
      {
         timerIsOn = FALSE;
      }
#else
    uint16 totalbytes = qq_total();
    uint8 numBytes;
    
      // 读  RdLen 个字节数据到 缓冲区 RdBuf， 返回读取到的有效数据长度
    if(totalbytes > 0 &&  simpleBLEChar6DoWrite)
    {            
        attPrepareWriteReq_t AttReq;           
        numBytes = ( (totalbytes > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : totalbytes);
        
//        LCD_WRITE_STRING_VALUE( "s=", numBytes, 10, HAL_LCD_LINE_1 );        
        
        AttReq.pValue = GATT_bm_alloc( simpleBLEConnHandle, ATT_WRITE_REQ, numBytes, NULL );
        if ( AttReq.pValue != NULL )
        {   
            AttReq.handle = simpleBLECharHd6;
            AttReq.len = numBytes;
            AttReq.sig = 0;
            AttReq.cmd = 0;

            //osal_memcpy(AttReq.pValue,buf,AttReq.len);
            qq_read(AttReq.pValue, AttReq.len);

//            LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
            //GATT_WriteCharValue( simpleBLEConnHandle, &AttReq, simpleBLETaskId );
            GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );

            simpleBLEChar6DoWrite = FALSE;

//            LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
        }
        else
        {
            LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
        }   
    }

      if(qq_total() > 0)
      {
         timerIsOn = TRUE;
         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 1);
      }
      else
      {
         timerIsOn = FALSE;
      }
#endif
      return ( events ^ SBP_DATA_EVT );
  }

  if ( events & SBP_UART_EVT )
  {      
      if(FALSE == timerIsOn)
      {
//         osal_set_event( simpleBLETaskId, SBP_DATA_EVT );
         osal_start_timerEx( simpleBLETaskId, SBP_DATA_EVT, 8);
      }      
      return ( events ^ SBP_UART_EVT );
  }  

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    simpleBLE_performPeriodicTask();

    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      if ( !simpleBLERssi )
      {
        simpleBLERssi = TRUE;
        GAPCentralRole_StartRssi( simpleBLEConnHandle, DEFAULT_RSSI_PERIOD );
        //LCD_WRITE_STRING( "RSSI Start...", HAL_LCD_LINE_1 );
      }
    }
    else
    {
        if(simpleBLERssi)
        {
            GAPCentralRole_CancelRssi( simpleBLEConnHandle );        
            //LCD_WRITE_STRING( "RSSI Cancelled", HAL_LCD_LINE_1 );
        }      
    }

    return (events ^ SBP_PERIODIC_EVT);
  }


  if ( events & START_DISCOVERY_EVT )
  {
//    LCD_WRITE_STRING("START_DISCOVERY_EVT", HAL_LCD_LINE_1 );
//    LCD_WRITE_STRING_VALUE( "line=", __LINE__, 10, HAL_LCD_LINE_1 );
    simpleBLECentralStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }

   // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
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
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
    /*
    按键处理公共函数， 主机与从机都是运行这个函数，
    注意每次启动不是主机就是从机，不是同时是主机与从机的， 所以他们不冲突的
    */
    simpleBLE_HandleKeys(keys);
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    goto EXIT;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      //uint8 status = pMsg->msg.errorRsp.errCode;
      
//      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a successful read, display the read value
      //uint8 valueRead = pMsg->msg.readRsp.pValue[0];

//      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
    }
    
    //simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
      //uint8 status = pMsg->msg.errorRsp.errCode;
      
//      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
      //LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );      
      // 这个变量用于表明上一次写数据到从机已经成功， 可用于判断写数据时的判断， 以确保数据的完整性
      simpleBLEChar6DoWrite = TRUE;
    }
    
    //simpleBLEProcedureInProgress = FALSE;    

  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  else if ( ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) )   //通知
  {
    if( pMsg->msg.handleValueNoti.handle == simpleBLECharHd6/*0x0035*/)   //CHAR6的通知  串口打印
    {
        if(simpleBLE_CheckIfUse_Uart2Uart())     //使用透传模式时才透传
        {        
            NPI_WriteTransport(pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len ); 

            // 这里可以处理一下数据，比如发命令点灯
            // MT 命令处理 函数
            simpleBLE_MT_CMD_Handle(pMsg->msg.handleValueNoti.pValue, pMsg->msg.handleValueNoti.len );
        }
    }
  }  

EXIT:
//  LCD_WRITE_STRING_VALUE( "free line=", __LINE__, 10, HAL_LCD_LINE_1 );
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    //LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
    simpleBle_SetRssi(rssi);
}

// 显示下一个从设备的地址  nextFalg=true则显示下一个地址，否则显示当前地址
void simpleBLECentraDisplaNextPeriAddr(bool nextFalg)
{
    char str[24];

    if(nextFalg)
    {
        simpleBLEScanIdx++;
        simpleBLEScanIdx %= simpleBLEScanRes;
    }
        
   if ( simpleBLEScanRes > 0 )
   {
        sprintf(str, "[%d]: %s",simpleBLEScanIdx, bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr));
        LCD_WRITE_STRING(str, HAL_LCD_LINE_3);  
        //LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ), HAL_LCD_LINE_3);  
   }
}

// 获取当前从设备地址的索引号
uint8 simpleBLECentraGetAddrId()
{
   return simpleBLEScanIdx;
}


/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        bool ifDoConnect = FALSE;

        // discovery complete
        simpleBLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
                                10, HAL_LCD_LINE_1 );
        if ( simpleBLEScanRes > 0 )
        {
            //simpleBLEScanIdx = 0;

            LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
//            LCD_WRITE_STRING_VALUE( "simpleBLEState = ", simpleBLEState, 10, HAL_LCD_LINE_2 );
            //这里可以决定连接到所搜索到的哪一个从机， 这里可以进行从机地址码的判断， 比如， 可以连接到最近一次连接成功的mac地址的从机
            //HalLedSet(HAL_LED_3, HAL_LED_MODE_ON );   //开LED3
            if ( simpleBLEState == BLE_STATE_IDLE )
            {           
              uint8 addrType;
              uint8 *peerAddr;
              uint8 i;
              uint8 Addr[MAC_ADDR_CHAR_LEN];
              //bool ret = FALSE;;

              simpleBLECentraDisplaNextPeriAddr(FALSE);
//              for(i = 0; i < simpleBLEScanRes; i++)
//              {
//                  LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[i].addr ), HAL_LCD_LINE_2+i );  
//              }  
#if 1         
//            LCD_WRITE_STRING_VALUE( "g_Central_connect_cmd = ", g_Central_connect_cmd, 10, HAL_LCD_LINE_2 );
            // 打印从机地址， 格式:  OK+DISC:123456789012
            if(g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_DISC)
            {
                char strTemp[24] = {0};
                for(i = 0; i < simpleBLEScanRes; i++)
                {
                    sprintf(strTemp, "OK+DISC:");
                    osal_memcpy(strTemp + 8,  bdAddr2Str( simpleBLEDevList[i].addr )+2, MAC_ADDR_CHAR_LEN);
                    strTemp[20] = '\r'; 
                    strTemp[21] = '\n'; 
                    strTemp[22] = 0;
                    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                }
                
                // 搜索完成后返回 OK+DISCE。
                sprintf(strTemp, "OK+DISCE\r\n");
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }
            else if(g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_CONNL
            || g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_CONN )
            {//连接指定最后成功的从机
                // 连接从机
                if(simpleBLE_GetToConnectFlag(Addr))
                {
                  uint8 tempstr[13] = {0};
                  osal_memcpy(tempstr, Addr, 12);
                  LCD_WRITE_STRING_VALUE( "ConAddr ", 0, 10, HAL_LCD_LINE_2 );
                  LCD_WRITE_STRING( (char *)tempstr, HAL_LCD_LINE_2 );  

                  // 连接指定的mac 地址的从机设备  
                  //simpleBLEScanIdx = 0;
                  for(i = 0; i < simpleBLEScanRes; i++)
                  {
//                      LCD_WRITE_STRING_VALUE( "peerAddr ", i, 10, HAL_LCD_LINE_2 );
//                      LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[i].addr ), HAL_LCD_LINE_2 );  

                      // connect to current device in scan result 
                      peerAddr = simpleBLEDevList[i].addr;
                      addrType = simpleBLEDevList[i].addrType;

                      if(osal_memcmp(Addr, bdAddr2Str( simpleBLEDevList[i].addr )+2, MAC_ADDR_CHAR_LEN))
                      {
                        //simpleBLEScanIdx = i;
                        ifDoConnect = TRUE;
                        break;                          
                      }
                  }
                } 
            }
            else if(g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_CON)
            {//连接指定地址的从机
                if(simpleBLE_GetToConnectFlag(Addr))
                {
                  uint8 tempstr[13] = {0};
                  osal_memcpy(tempstr, Addr, 12);
//                  LCD_WRITE_STRING_VALUE( "ConAddr ", 0, 10, HAL_LCD_LINE_2 );
//                  LCD_WRITE_STRING( (char *)tempstr, HAL_LCD_LINE_2 );  

                  // 连接指定的mac 地址的从机设备  
                  //simpleBLEScanIdx = 0;
                  for(i = 0; i < simpleBLEScanRes; i++)
                  {
//                      LCD_WRITE_STRING_VALUE( "peerAddr ", i, 10, HAL_LCD_LINE_2 );
//                      LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[i].addr ), HAL_LCD_LINE_2 );  

                      // connect to current device in scan result 
                      peerAddr = simpleBLEDevList[i].addr;
                      addrType = simpleBLEDevList[i].addrType;

                      if(osal_memcmp(Addr, bdAddr2Str( simpleBLEDevList[i].addr )+2, MAC_ADDR_CHAR_LEN))
                      {
                        //simpleBLEScanIdx = i;
                        ifDoConnect = TRUE;
                        break;                          
                      }
                  }
                } 
            }

#endif
              if(ifDoConnect)
              {
                  simpleBLEState = BLE_STATE_CONNECTING;

                 NPI_WriteTransport("Connecting\r\n", 12); 
                 
//                 LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
//                 LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );  
              
                 GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                                DEFAULT_LINK_WHITE_LIST,
                                                addrType, peerAddr );
              }
              else
              {
//                LCD_WRITE_STRING( "Continue scanning", HAL_LCD_LINE_1 );
                
                // 继续扫描
                simpleBLEScanning  = FALSE;
                //simpleBLEScanRes = 0;                
                //simpleBLEStartScan();
              }
            }
            
            //HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF ); 

            // initialize scan index to last device
            //simpleBLEScanIdx = simpleBLEScanRes;
        }

        if( ifDoConnect == FALSE )
        {
            // 判断 如如果设置过 AT+IMME0 的， 扫描不到以前连接过的从机时， 继续扫描
            if(1 == sys_config.workMode)
            {
                g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_CONNL;

                // 开始扫描
                simpleBLEStartScan();
            }
        }
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          //simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if ( simpleBLECharHdl == 0 )
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
                    
          LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );  

          // 增加从机地址， 注意， 需要连接成功后， 再增加改地址
          //simpleBLE_SetPeripheralMacAddr((uint8 *)(bdAddr2Str( pEvent->linkCmpl.devAddr ))+2);
          simpleBLE_SetPeripheralMacAddr((uint8 *)bdAddr2Str(simpleBLEDevList[simpleBLEScanIdx].addr)+2);
          HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF );   
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          //simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          simpleBLECentralCanSend = FALSE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
          //HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF );   //关LED3
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        //simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        simpleBLECharHdl = 0;
        //simpleBLEProcedureInProgress = FALSE;
        simpleBLECentralCanSend = FALSE;

        NPI_WriteTransport("Disconnected\r\n", 14);
        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                10, HAL_LCD_LINE_2 );
        //HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF );   //关LED3


        //这里连接失败后， 可以尝试执行重启或者继续扫描从机
        simpleBLEScanRes = 0;
//        LCD_WRITE_STRING_VALUE( "simpleBLEScanning=", simpleBLEScanning, 10, HAL_LCD_LINE_1 );
//        LCD_WRITE_STRING_VALUE( "simpleBLEScanRes=", simpleBLEScanRes, 10, HAL_LCD_LINE_1 );

        simpleBLEScanning = 0;
        simpleBLEScanRes = 0;
        simpleBLEStartScan();
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
         LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1);    
         NPI_WriteTransport("Param Update\r\n", 14);

         osal_start_timerEx( simpleBLETaskId, SBP_CONNECT_EVT, 1000 );

         g_sleepFlag = FALSE;
         osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  不睡眠，功耗很高的        
      }
      break;
      
    default:
      break;
  }
  return ( TRUE );
}

bool simpleBLEConnect(uint8 index)
{
    uint8 addrType;
    uint8 *peerAddr;
    uint8 i = index;

    if(i >= simpleBLEScanRes)
        return FALSE;
    if(i >= DEFAULT_MAX_SCAN_RES)
        return FALSE;
    
    simpleBLEScanIdx = index;
    // connect to current device in scan result 
    peerAddr = simpleBLEDevList[i].addr;
    addrType = simpleBLEDevList[i].addrType;

    if(simpleBLEState == BLE_STATE_IDLE)
    {
        simpleBLEState = BLE_STATE_CONNECTING;

        NPI_WriteTransport("Connecting\r\n", 12); 
        
        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 );  

        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                DEFAULT_LINK_WHITE_LIST,
                                addrType, peerAddr );

        return TRUE;
    }
    else
    {
//        LCD_WRITE_STRING( "BLE_STATE not in IDLE", HAL_LCD_LINE_1 );
    }
    return FALSE;
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
//#if (HAL_LCD == TRUE)

  uint32  passcode = simpleBle_GetPassword();
  uint8   str[7];

  // Create random passcode
  //LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  //passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  LCD_WRITE_STRING_VALUE( ">>>passcode=", passcode, 10, HAL_LCD_LINE_1 );
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
//#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
  
  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;

//  LCD_WRITE_STRING_VALUE( "---DiscState=", simpleBLEDiscState, 10, HAL_LCD_LINE_1 );
  
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      simpleBLESvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      simpleBLESvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);

      // 把每一个handle的值都打印出来
//      LCD_WRITE_STRING_VALUE( "StartHdl=", simpleBLESvcStartHdl, 10, HAL_LCD_LINE_1 );
//      LCD_WRITE_STRING_VALUE( "EndHdl=", simpleBLESvcEndHdl, 10, HAL_LCD_LINE_1 );

//      simpleBLECharHd6 = simpleBLESvcStartHdl;
      
      for(int i = simpleBLESvcStartHdl; i < simpleBLESvcStartHdl+10; i++)  
      {
//        LCD_WRITE_STRING_VALUE( "handle = ", i, 10, HAL_LCD_LINE_1 );
      }
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR6_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR6_UUID);

        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
       simpleBLECharHd6 = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                      pMsg->msg.readByTypeRsp.pDataList[1]);
      
      LCD_WRITE_STRING( "Simple Svc Found", HAL_LCD_LINE_1 );
      //simpleBLEProcedureInProgress = FALSE;
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "pMsg->method=", pMsg->method, 10, HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( "ERR in Simple Svc Found", HAL_LCD_LINE_1 );
    }
    
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;

    
  }    
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
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
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

