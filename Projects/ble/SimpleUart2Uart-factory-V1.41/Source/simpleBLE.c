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
#include "peripheral.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "npi.h"
#include "osal_snv.h"
#include "simpleBLE.h"
#include "stdio.h"
#include "string.h"
#include "hal_adc.h"
#include "amomcu_buffer.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "VibrativeSensor.h"
#include "DHT11.h"

#if 1
SYS_CONFIG sys_config;
bool g_sleepFlag = TRUE;    //睡眠标志
uint8 uart_sleep_count = 0; // 睡眠计数器

bool g_rssi_flag = false;       //是否开启测距
    
extern gaprole_States_t gapProfileState;   // 从机连接状态
// Connection handle
extern uint16 gapConnHandle;

BLE_CENTRAL_CONNECT_CMD g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_NULL;

static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events );
static bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length);
static void simpleBLE_SendMyData_ForTest();
#endif

#if 1
// 该函数延时时间为1ms， 用示波器测量过， 稍有误差， 但误差很小  --amomcu.com
void simpleBLE_Delay_1ms(int times)
{
  while(times--)
  {
      int i=0;
      for(i=1500;i>0;i--)
      {
    	  asm("nop");
      }
  }
}

// 字符串对比
uint8 str_cmp(uint8 *p1,uint8 *p2,uint8 len)
{
  uint8 i=0;
  while(i<len){
    if(p1[i]!=p2[i])
      return 0;
    i++;
  }
  return 1;
}

// 字符串转数字
uint32 str2Num(uint8* numStr, uint8 iLength)
{
    uint8 i = 0;
    int32 rtnInt = 0;
 
    /* 
          为代码简单，在确定输入的字符串都是数字的
          情况下，此处未做检查，否则要检查
          numStr[i] - '0'是否在[0, 9]这个区间内
    */
    for(; i < iLength && numStr[i] != '\0'; ++i)
        rtnInt = rtnInt * 10 + (numStr[i] - '0');    
 
    return rtnInt;
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
#define B_ADDR_STR_LEN                        15

  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}
#endif

// 保存所有数据到nv flash
void simpleBLE_WriteAllDataToFlash()
{   // 写所有参数
    osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config); 
}

// 读取自定义的 nv flash 数据  -------未使用到
void simpleBLE_ReadAllDataToFlash()
{
    int8 ret8 = osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
}

//flag: PARA_ALL_FACTORY:  全部恢复出厂设置
//flag: PARA_PARI_FACTORY: 清除配对信息
void simpleBLE_SetAllParaDefault(PARA_SET_FACTORY flag)    
{
    if(flag == PARA_ALL_FACTORY)
    {
        //sys_config.baudrate = HAL_UART_BR_9600;  
        sys_config.baudrate = HAL_UART_BR_115200;  
        sys_config.parity = 0;  
        sys_config.stopbit = 0;  

        sys_config.mode = BLE_MODE_SERIAL;         //工作模式 0:透传 ， 1: 直驱 , 2: iBeacon

        sprintf((char*)sys_config.name, DEV_NAME_DEFAULT);  //设备名称

        sys_config.role = BLE_ROLE_PERIPHERAL;         //主从模式, 默认从机
        //sys_config.role = BLE_ROLE_CENTRAL;

        sprintf((char*)sys_config.pass, "000000");      //密码
        sys_config.type = 0;                            //鉴权模式
        osal_memset(sys_config.mac_addr, 0, sizeof(sys_config.mac_addr));

        sys_config.ever_connect_peripheral_mac_addr_conut = 0;
        sys_config.ever_connect_peripheral_mac_addr_index = 0;        
        //曾经成功连接过的从机地址
        osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*MAC_ADDR_CHAR_LEN);

        sys_config.try_connect_time_ms = 0;       // 尝试连接时间---目前无效 

        sys_config.rssi = 0;                      //  RSSI 信号值

        sys_config.rxGain = HCI_EXT_RX_GAIN_STD;   //  接收增益强度
        sys_config.txPower = 0;                    //  发射信号强度

        sys_config.ibeacon_adver_time_ms = 500;
        //  模块工作类型  0: 立即工作， 1: 等待AT+CON 或 AT+CONNL 命令
        sys_config.workMode = 0;          
    }
    else if(flag == PARA_PARI_FACTORY)
    {
        //sprintf((char*)sys_config.pass, "000000");      //密码
        osal_memset(sys_config.mac_addr, 0, sizeof(sys_config.mac_addr));
        sys_config.ever_connect_peripheral_mac_addr_conut = 0;
        sys_config.ever_connect_peripheral_mac_addr_index = 0;        
        osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*MAC_ADDR_CHAR_LEN);
    }

    GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL ); //清除绑定信息

    simpleBLE_WriteAllDataToFlash();
}

// 打印所有存储到nv flash的数据， 方便调试代码
void PrintAllPara(void)
{
    char strTemp[32];
    
    sprintf(strTemp, "sys_config.baudrate = %d\r\n", sys_config.baudrate);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);
    
    sprintf(strTemp, "sys_config.parity = %d\r\n", sys_config.parity); 
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.stopbit = %d\r\n", sys_config.stopbit);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.mode = %d\r\n", sys_config.mode);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.name = %s\r\n", sys_config.name);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);
    
    sprintf(strTemp, "sys_config.role = %d\r\n", sys_config.role);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);
    
    sprintf(strTemp, "sys_config.pass = %s\r\n", sys_config.pass);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);
    
    sprintf(strTemp, "sys_config.type = %d\r\n", sys_config.type);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.mac_addr = %s\r\n", sys_config.mac_addr);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    //曾经成功连接过的从机地址
    //LCD_WRITE_STRING_VALUE( "addr_conut:", sys_config.ever_connect_peripheral_mac_addr_conut, 10, HAL_LCD_LINE_2 );

    for(int i = 0; i < sys_config.ever_connect_peripheral_mac_addr_conut; i++)
    {
        uint8 temp_addr[MAC_ADDR_CHAR_LEN+1] = {0};
        osal_memcpy(temp_addr, sys_config.ever_connect_mac_status[i], MAC_ADDR_CHAR_LEN);
        sprintf(strTemp, "[%d] = %s\r\n", i, temp_addr);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        simpleBLE_Delay_1ms(200);
    }
    
    sprintf(strTemp, "sys_config.try_connect_time_ms = %d\r\n", sys_config.try_connect_time_ms);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.rssi = %d\r\n", sys_config.rssi);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.txPower = %d\r\n", sys_config.txPower);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.ibeacon_adver_time_ms = %d12\r\n", sys_config.ibeacon_adver_time_ms);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);

    sprintf(strTemp, "sys_config.workMode = %d\r\n", sys_config.workMode);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    simpleBLE_Delay_1ms(100);
}

// 返回设备角色
//主从模式  0: 从机   1: 主机
BLE_ROLE GetBleRole()
{
    return sys_config.role;
}

// 判断蓝牙是否连接上
// 0: 未连接上
// 1: 已连接上
bool simpleBLE_IfConnected()
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
    {          
        return ( simpleBLEState == BLE_STATE_CONNECTED  );
    }
    else
    {
        return (gapProfileState == GAPROLE_CONNECTED);
    }
}

// 增加从机地址， 注意， 需要连接成功后， 再增加该地址
void simpleBLE_SetPeripheralMacAddr(uint8 *pAddr)
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
    {
       uint8 Addr[MAC_ADDR_CHAR_LEN];
       uint8 index;
       
       //LCD_WRITE_STRING_VALUE( "_conut:", sys_config.ever_connect_peripheral_mac_addr_conut, 10, HAL_LCD_LINE_2 );
       // 要增加从机地址， 需要先检查我们的地址列表里是否已经存有改地址， 如果已经有了， 那么无需再增加
       for(index = 0; index < sys_config.ever_connect_peripheral_mac_addr_conut; index++)        
       {
          if(simpleBLE_GetPeripheralMacAddr(index, Addr))
          {
             if(osal_memcmp(Addr, pAddr, MAC_ADDR_CHAR_LEN))//地址一样时直接返回
             {
                //LCD_WRITE_STRING_VALUE( "_index: return", index, 10, HAL_LCD_LINE_2 );
                //最新一次成功连接过的从机地址index， 用于针对AT+CONNL 这个指令
                sys_config.last_connect_peripheral_mac_addr_index = index;
                return;
             }
          }
          else
          {
             break;
          }
       }

       //LCD_WRITE_STRING_VALUE( "_index:", sys_config.last_connect_peripheral_mac_addr_index, 10, HAL_LCD_LINE_2 );
       
       //只增加这多的从机地址， 超出后, 覆盖最先的一个地址记录，列表里只保存最近的     MAX_PERIPHERAL_MAC_ADDR    个地址
       osal_memcpy(sys_config.ever_connect_mac_status[sys_config.ever_connect_peripheral_mac_addr_index], pAddr, MAC_ADDR_CHAR_LEN);

       //最新一次成功连接过的从机地址index， 用于针对AT+CONNL 这个指令
       sys_config.last_connect_peripheral_mac_addr_index = sys_config.ever_connect_peripheral_mac_addr_index;

       sys_config.ever_connect_peripheral_mac_addr_index++;
       // 注意下面这个的技巧用法
       sys_config.ever_connect_peripheral_mac_addr_index %= MAX_PERIPHERAL_MAC_ADDR; 

       // 只记录  MAX_PERIPHERAL_MAC_ADDR 个最新的地址
       if(sys_config.ever_connect_peripheral_mac_addr_conut < MAX_PERIPHERAL_MAC_ADDR) 
       {
         sys_config.ever_connect_peripheral_mac_addr_conut++; 
       }

       //LCD_WRITE_STRING_VALUE( "_conut2:", sys_config.ever_connect_peripheral_mac_addr_conut, 10, HAL_LCD_LINE_2 );

       // 保存地址 ， 以便重启机子后可以也使用 
       simpleBLE_WriteAllDataToFlash();
    }
}

// 读取从机地址, index < MAX_PERIPHERAL_MAC_ADDR
// 用于判断是否系统中已存有该Mac地址
/*
index: 应该是 < MAX_PERIPHERAL_MAC_ADDR,
*/
bool simpleBLE_GetPeripheralMacAddr(uint8 index, uint8 *pAddr)
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
    {  
        if(index < sys_config.ever_connect_peripheral_mac_addr_conut
        /*&& index < MAX_PERIPHERAL_MAC_ADDR*/ )
        {
            osal_memcpy(pAddr, sys_config.ever_connect_mac_status[index], MAC_ADDR_CHAR_LEN);
            return TRUE;
        }
    }
    return FALSE;
}


// 有按键按下，则启动为主机， 否则默认启动为从机
// 0 启动peripheral从设备， 1: 启动为 central
bool Check_startup_peripheral_or_central(void) 
{
    P0SEL &= ~0x02;     //设置P0.1为普通IO口  
    P0DIR &= ~0x02;     //按键接在P0.1口上，设P0.1为输入模式 
    P0INP &= ~0x02;     //打开P0.1上拉电阻

    if(0 == P0_1)// 有按键按下
    {
        // 10ms 去抖动 
        simpleBLE_Delay_1ms(10);
        if(0 == P0_1)// 有按键按下
        {
            return true;
        }
    }

    return false;
}


//开机时判断到按键按下3秒， 恢复出厂设置
//按键定义为  p0.7
void CheckKeyForSetAllParaDefault(void) 
{
#if 0  
    uint8 i;
    uint32 old_time  = 30; 

    P0SEL &= ~0x02;     //设置P0.1为普通IO口  
    P0DIR &= ~0x02;     //按键接在P0.1口上，设P0.1为输入模式 
    P0INP &= ~0x02;     //打开P0.1上拉电阻


    //判断3s 內按键都没有松开
    while(--old_time)
    {
        if(P0_1 == 0)
        {
            simpleBle_LedSetState(HAL_LED_MODE_ON);  
            simpleBLE_Delay_1ms(100);
        }
        else
        {
            simpleBle_LedSetState(HAL_LED_MODE_OFF);  
            return;
        }        
    }

    //判断3s 內按键都没有松开， 那么就行恢复出厂设置，下面会先闪灯三下再恢复出厂设置然后重启
    if(old_time == 0)
    {
        simpleBLE_SetAllParaDefault(PARA_ALL_FACTORY);
        for(i = 0; i < 6; i++)    
        {
            simpleBle_LedSetState(HAL_LED_MODE_ON);  
            simpleBLE_Delay_1ms(100);
            simpleBle_LedSetState(HAL_LED_MODE_OFF);
            simpleBLE_Delay_1ms(100);
        }   
        // 重启， 实际上是利用看门狗了
        HAL_SYSTEM_RESET();     
    }
#endif

#if 0
    if(P0_1 == 0)// 有按键按下
    {
        simpleBle_LedSetState(HAL_LED_MODE_ON);  
        simpleBLE_Delay_1ms(100);
    }
    else
    {
        simpleBle_LedSetState(HAL_LED_MODE_OFF);  
        return;
    }        
#endif
}

// 串行口 uart 初始化
void simpleBLE_NPI_init(void)
{
#if 1    
    NPI_InitTransportEx(simpleBLE_NpiSerialCallback, sys_config.baudrate, 
    sys_config.parity, sys_config.stopbit );
#else
    NPI_InitTransport(simpleBLE_NpiSerialCallback);
#endif

    // 开机打印主机还是从机
    if(GetBleRole() == BLE_ROLE_CENTRAL)
    {
        NPI_WriteTransport("Hello World Central\r\n",21);
    }
    else
    {
        NPI_WriteTransport("Hello World Peripheral\r\n",24);
    }
}

// 设置接收增益
void UpdateRxGain(void)
{
    // HCI_EXT_SetRxGainCmd()是用来设置发射功率的. 
    // rxGain - HCI_EXT_RX_GAIN_STD, HCI_EXT_RX_GAIN_HIGH
    HCI_EXT_SetRxGainCmd( HCI_EXT_RX_GAIN_STD );
}

// 设置发射功率
void UpdateTxPower(void)
{
        /*
#define LL_EXT_TX_POWER_MINUS_23_DBM                   0
#define LL_EXT_TX_POWER_MINUS_6_DBM                    1
#define LL_EXT_TX_POWER_0_DBM                          2
#define LL_EXT_TX_POWER_4_DBM                          3
        */
    // HCI_EXT_SetTxPowerCmd()是用来设置发射功率的. 有-23dbm, -6dbm, 0 dbm, +4dbm四个级别. 
//    uint8 txPower;
//#if defined(CC2541)
    HCI_EXT_SetTxPowerCmd(3 - sys_config.txPower);
//#endif
}

// 设置led灯的状态
void simpleBle_LedSetState(uint8 onoff)
{
  HalLedSet( HAL_LED_1, onoff);  //led常亮

  P0DIR |= 0x60;  // P0.6定义为输出
  P0_6 = onoff;  
}

#if 1
static float GUA_CalcDistByRSSI(int rssi)    
{    
    uint8 A = 49;  
    float n = 3.0;  
      
    int iRssi = abs(rssi);    
    float power = (iRssi-A)/(10*n);         
    return pow(10, power);    
}

// 求滑动平均值
#define DIST_MAX   5
int nDistbuf[DIST_MAX];
uint8 index = 0;

static int dist_filer(int dist)
{
    int i = 0;
    int sum = 0;
    int max = 0;
    int min = 1000;
    if(index == DIST_MAX)
    {
         static int index2 = 0;
         nDistbuf[index2++] = dist;
         index2 %= DIST_MAX;

         // 去掉最大最小值, 再求平均
         
         for(i =0; i< DIST_MAX; i++)
         {
            if(max < nDistbuf[i])
               max = nDistbuf[i];
            if(min > nDistbuf[i])
               min = nDistbuf[i];
            
            sum += nDistbuf[i];
         }
         return (sum-max-min)/(DIST_MAX-2);
    }
    else
    {
        nDistbuf[index++] = dist;
        return 0;
    }
}
#endif

// 保存RSSI 到系统变量
void simpleBle_SetRssi(int8 rssi)
{
    sys_config.rssi = rssi;

    if(simpleBLE_IfConnected())
    {
        char str[32];    

        float nfDist = GUA_CalcDistByRSSI(rssi);         //通过算法获得r，单位为m  
        int nDist = (int)(nfDist * 100);                    //将r的数值放大100倍，单位为cm  
        sprintf(str, "Rssi=%2d,%4dCM\r\n", (uint8) (-rssi), dist_filer(nDist));

        if(g_rssi_flag)
        {
            NPI_WriteTransport((uint8*)str, strlen(str));

            //发送到对端。 比如手机
            qq_write((uint8*)str, strlen(str));
            // 启动事件，然后在事件中再启动定时器定时检测数据并发送到网络
            osal_set_event( simpleBLETaskId, SBP_UART_EVT );     
        }

        LCD_WRITE_STRING(str, HAL_LCD_LINE_5 );
    }  
}

// 串口打印密码  -----测试用----
void simpleBle_PrintPassword()
{
    char strTemp[24] = {0};
    
    sprintf(strTemp, "Password:%s\r\n", sys_config.pass);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));
}

// 获取设备名称
uint8* simpleBle_GetAttDeviceName()
{
    return sys_config.name;
}

// 主机是否记录了从机地址
bool simpleBle_IFfHavePeripheralMacAddr( void )
{
    if(sys_config.ever_connect_peripheral_mac_addr_conut > 0)
        return TRUE;
    else
        return TRUE;
}

// 定时器任务定时执行函数， 用于设置led的状态----也可以增加一个定时器来做
void simpleBLE_performPeriodicTask( void )
{
    static uint8 count = 0;

//    if(simpleBLE_CheckIfUse_iBeacon())  
//    {		
//        static attHandleValueNoti_t pReport;
//        pReport.len = 2;
//        pReport.handle = 0x2E;
//        pReport.pValue = GATT_bm_alloc( gapConnHandle, ATT_WRITE_REQ, pReport.len, NULL );
//        if(pReport.pValue != NULL)
//        {
//            pReport.pValue[0] = 'a';  
//            pReport.pValue[1] = 'b';
//            GATT_Notification(gapConnHandle, &pReport, FALSE );            
//        }
//        
//        simpleBle_LedSetState(HAL_LED_MODE_TOGGLE);           //取反		
//        return;
//    }
    
    /*
    连线前，
        主机未记录从机地址时，每秒亮100ms；
        主机记录从机地址时，每秒亮900ms；
        从机每2秒亮1秒。
    连线后，
        主机与从机均为，LED每5秒亮100毫秒。
    */
    if(!simpleBLE_IfConnected())
    {    
        if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
        {     
            if(simpleBle_IFfHavePeripheralMacAddr() == FALSE)//未记录地址
            {
                if(count == 0)
                {
                    simpleBle_LedSetState(HAL_LED_MODE_ON);  
                }
                else if(count == 1)
                {
                    simpleBle_LedSetState(HAL_LED_MODE_OFF);
                }
            }
            else
            {
                if(count == 0)
                {
                    simpleBle_LedSetState(HAL_LED_MODE_ON);  
                } 
                else if(count == 9)
                {
                    simpleBle_LedSetState(HAL_LED_MODE_OFF);
                }
            }                    
            count++;
            count %= 10;     
        }  
        else//从机
        {
            if(count == 0)
            {
                simpleBle_LedSetState(HAL_LED_MODE_OFF);  
            } 
            else if(count == 10)
            {
                simpleBle_LedSetState(HAL_LED_MODE_ON);
            }

            count++;
            count %= 20;
        }

#if defined(USE_DISPLAY_KEY_VALUE)  // 测试按键专用，显示5向按键值
        SimpleBLE_DisplayTestKeyValue();       
#endif
    }
    else// 连接后 主机与从机均为，LED每5秒亮100毫秒。(如果想省电， 可以不点灯)
    {
        
        if(count == 0)
        {
            simpleBle_LedSetState(HAL_LED_MODE_ON);  
        } 
        else if(count == 1)
        {
            simpleBle_LedSetState(HAL_LED_MODE_OFF);
        }
        count++;
        count %= 50; 

        // 隔1min发送一次温湿度值
        simpleBLE_SendMyData_ForTest();
    }
}

// 获取鉴权要求, 0: 连接不需要密码,  1: 连接需要密码
bool simpleBle_GetIfNeedPassword()
{
    return sys_config.type;
}

// 获取连接密码
uint32 simpleBle_GetPassword()
{
    uint32 passcode;   
    
    passcode = str2Num(sys_config.pass, 6);
    passcode %= 1000000;

    return passcode;
}

// 判断是否是 iBeacon 广播模式
bool simpleBLE_CheckIfUse_iBeacon()
{
    return (sys_config.mode == BLE_MODE_iBeacon);
}

// 判断是否使能串口透传
bool simpleBLE_CheckIfUse_Uart2Uart()
{
    return (sys_config.mode == BLE_MODE_SERIAL);
}

// 判断是输入的形参-地址是否是需要去连接的地址，如果是， 返回真， 否则返回假
bool simpleBLE_GetToConnectFlag(uint8 *Addr)
{
    if(g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_DISC)
    {
        return FALSE;
    }
    else if((g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_CONNL)
    || (g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_CONN))
    {
        if(sys_config.ever_connect_peripheral_mac_addr_conut > 0)
        {
            osal_memcpy(Addr, sys_config.ever_connect_mac_status[sys_config.last_connect_peripheral_mac_addr_index], MAC_ADDR_CHAR_LEN);            
            return TRUE; 
        }
    }
    else if(g_Central_connect_cmd  == BLE_CENTRAL_CONNECT_CMD_CON)
    {
        osal_memcpy(Addr, sys_config.connect_mac_addr, MAC_ADDR_CHAR_LEN);            
        return TRUE; 
    }

    return FALSE;
}

// 设置 iBeacon 的广播间隔
uint32 simpleBLE_GetiBeaconAdvertisingInterral()
{
    return sys_config.ibeacon_adver_time_ms;    
}

#if 1
// 串口回调函数， 下面把该回调函数里实现的功能讲解一下
/*
1, 思路:  当串口收到数据后，就会马上调用以下回调函数，在实际测试中发现，此回调
函数调用频繁， 如果你不执行NPI_ReadTransport函数进行读取， 那么这个回调函数就会
频繁地被执行，但是，你通过串口发送一段数据， 你本意是想处理这一完整一段的数据，所以，
我们在下面引入了时间的处理方法， 也即接收的数据够多或者超时，就读取一次数据， 
然后根据当前的状态决定执行，如果没有连接上，就把所有数据当做AT命令处理， 如果连接
上了，就把数据送到对端。
*/

//uart 回调函数
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port;

    static uint32 old_time;     //老时间
    static uint32 old_time_data_len = 0;     //老时间是的数据长度    
    uint32 new_time;            //新时间
    bool ret;
    uint8 readMaxBytes = SIMPLEPROFILE_CHAR6_LEN;

    if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))   //串口有数据
    {
        (void)port;
        /*uint8*/uint16 numBytes = 0;

        uart_sleep_count = 0;
        
#if 1
        if(TRUE == g_sleepFlag)//正在睡眠中...把数据读取出来丢弃掉即可
        {
            numBytes = NPI_RxBufLen();           //读出串口缓冲区有多少字节
            if(numBytes > 0)
            {
                // 注意本函数会连续调用， 所以下面我只读取部分数据都是可以的
                numBytes = (numBytes > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : numBytes;                    
                uint8 *buffer = osal_mem_alloc(numBytes);
                NPI_ReadTransport(buffer,numBytes);    //释放串口数据    
                osal_mem_free(buffer);
            }
            
            return;
        }
        else
        {
            numBytes = NPI_RxBufLen();           //读出串口缓冲区有多少字节
        }
#else
        numBytes = NPI_RxBufLen();           //读出串口缓冲区有多少字节
#endif

#if 0 //for test
        uint8 strTemp[10];
        sprintf((char *)strTemp, "[%d]", numBytes);
        NPI_WriteTransport(strTemp, osal_strlen((char *)strTemp)); 
#endif        
        if(numBytes == 0)
        {
            //LCD_WRITE_STRING_VALUE( "ERROR: numBytes=", numBytes, 10, HAL_LCD_LINE_1 );
            old_time_data_len = 0;
            return;
        }
        if(old_time_data_len == 0)
        {
            old_time = osal_GetSystemClock(); //有数据来时， 记录一下
            old_time_data_len = numBytes;
        }
        else
        {
            // 注意: 未连接上时， 有些AT 命令比较长， 所以需要开辟较大的缓冲区
            //       连接上以后， 收到每一能发送的数据不超过 SIMPLEPROFILE_CHAR6_LEN 的字节数的限制
            //       因此，这里要限制一下
            if(!simpleBLE_IfConnected())
            {
               readMaxBytes = 22 ;    //这个值， 一般设置成 AT 命令中最长的字节数即可， (包含"\r\n" 计数)
            }
            else
            {
               readMaxBytes = SIMPLEPROFILE_CHAR6_LEN;
            }

            
            new_time = osal_GetSystemClock(); //当前时间
            
            if( (numBytes >= readMaxBytes) 
                || ( (new_time - old_time) > 20/*ms*/))
            {
                uint8 sendBytes = 0;
                uint8 *buffer = osal_mem_alloc(readMaxBytes);

                if(!buffer)
                {
                    NPI_WriteTransport("FAIL", 4); 
                    return;
                }
                
                // 
                if(numBytes > readMaxBytes)
                {
                    sendBytes = readMaxBytes;
                }
                else
                {
                    sendBytes = numBytes;
                }

                if(!simpleBLE_IfConnected())
                {
                    //numBytes = NpiReadBuffer(buf, sizeof(buf));
                    //NpiClearBuffer();
                    NPI_ReadTransport(buffer,sendBytes);    //释放串口数据    
                    
                    if(sendBytes > 2 
                    && buffer[sendBytes-2] == '\r' 
                    && buffer[sendBytes-1] == '\n')
                    {//检测到 \r\n 结束的字符串， 表明是 AT 命令
                        ret = simpleBLE_AT_CMD_Handle(buffer, sendBytes);
                    }
                    else
                    {
                        ret = FALSE;
                    }
                    
                    if(ret == FALSE)
                    {
                        char strTemp[12];
                        //参数错误， 直接返回 "ERROR\r\n"， 不做任何参数更改
                        sprintf(strTemp, "ERROR\r\n");
                        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                    }
                }
                else
                {
                    NPI_ReadTransport(buffer,sendBytes);    //释放串口数据   
                    qq_write(buffer, sendBytes);

                    // 启动事件，然后在事件中再启动定时器定时检测数据并发送到网络
                    osal_set_event( simpleBLETaskId, SBP_UART_EVT );                    
                }

                old_time = new_time;
                old_time_data_len = numBytes - sendBytes;


                osal_mem_free(buffer);
            }                
        }    
    }
}



#endif

#if 1
// AT 命令处理 函数
bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length)
{
    bool ret = TRUE;
    char strTemp[64];
    uint8 i;
    uint8 temp8;  
    bool restart = FALSE;
 
    //NPI_WriteTransport((uint8*)pBuffer, length); 
    // 1、测试
    if((length == 4) && str_cmp(pBuffer, "AT\r\n", 4))//AT    
    {
        sprintf(strTemp, "OK\r\n");
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // 打印设置的参数  for test only
    else if((length == 8) && str_cmp(pBuffer, "AT+ALL\r\n", 8))//AT    
    {
        PrintAllPara();
    }
    // 2、查询、设置波特率
    else if((length == 10) && str_cmp(pBuffer, "AT+BAUD", 7))
    {
        /*
        发送：AT+BAUD2 
        返回：OK+Set:2 
        0---------9600 
        1---------19200 
        2---------38400 
        3---------57600 
        4---------115200
        */
        switch(pBuffer[7])
        {
        case '?':  //查询当前波特率
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.baudrate);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':  //查询设置新的波特率
            sys_config.baudrate = pBuffer[7] - '0';
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.baudrate);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:
            ret = FALSE;            
            break;
        }        
    }
    // 3、设置串口校验
    else if((length == 10) && str_cmp(pBuffer, "AT+PARI", 7))
    {   
        // 不做该功能
        ret = FALSE;            
    }    
    // 4、设置停止位
    else if((length == 10) && str_cmp(pBuffer, "AT+STOP", 7))
    {
        // 不做该功能
        ret = FALSE;            
    }       
    // 5. 设置模块工作模式
    else if((length == 10) && str_cmp(pBuffer, "AT+MODE", 7))
    {
        /*
        Para: 0 ~ 1
        0: 开启串口透传模式
        1: 关闭串口透传模式
        2: iBeacon 广播模式
        Default: 0 
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.mode);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
        case '2':
            sys_config.mode = pBuffer[7] - '0';            
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.mode);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }          
    // 6、查询、设置设备名称
    else if((length >=10 && length <= 20) && str_cmp(pBuffer, "AT+NAME", 7))
    {
        /*
        Para1：设备名称
        最长 11 位数字或字母，
        含中划线和下划线，不建
        议用其它字符。
        Default：DEV_NAME_DEFAULT(见宏定义)
        */
        //int nameLen = length - 7;
        
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%s\r\n", sys_config.name);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        default:
            osal_memset(sys_config.name, 0, sizeof(sys_config.name));
            osal_memcpy(sys_config.name, pBuffer + 7, length - 7);
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%s\r\n", sys_config.name);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        }        
    }    
    //7. 恢复默认设置(Renew)
    else if((length == 10) && str_cmp(pBuffer, "AT+RENEW", 8))
    {
        sprintf(strTemp, "OK+RENEW\r\n");
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        
        simpleBLE_SetAllParaDefault(PARA_ALL_FACTORY);

        restart = TRUE;  //直接重启即可
    }
    //8. 模块复位，重启(Reset)
    else if((length == 10) && str_cmp(pBuffer, "AT+RESET", 8))
    {
        restart = TRUE;  //直接重启即可
    }
    // 9、查询、设置主从模式
    else if((length == 10) && str_cmp(pBuffer, "AT+ROLE", 7))
    {
        /*
        Para1: 0 ~ 1 
        1: 主设备
        0: 从设备
        Default: 0 
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.role);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
            temp8 = pBuffer[7] - '0';            
            if(temp8 == 0)
            {
              sys_config.role = BLE_ROLE_PERIPHERAL;
            }
            else
            {
              sys_config.role = BLE_ROLE_CENTRAL;
            }
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.role);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }    
    // 10、 查询、设置配对密码
    else if(((length == 10) && str_cmp(pBuffer, "AT+PASS?", 8))
        || ((length == 15) && str_cmp(pBuffer, "AT+PASS", 7)))
    {
        /*
        Para1: 000000~999999 
        Default：000000
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+PASS:%s\r\n", sys_config.pass);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        default:
            osal_memcpy(sys_config.pass, pBuffer + 7, 6);
            sys_config.pass[6] = 0;
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%s\r\n", sys_config.pass);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));
            break;
        }
    }   
    // 11、 设置模块鉴权工作类型
    else if((length == 10) && str_cmp(pBuffer, "AT+TYPE", 7))
    {
        /*
        Para: 0 ~ 1 
        0: 连接不需要密码
        1: 连接需要密码
        Default: 0 
        */
        switch(pBuffer[7])
        {
        case '?':  
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.type);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
            sys_config.type = pBuffer[7] - '0';            
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.type);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //直接重启即可
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }       
    // 12、 查询本机 MAC 地址
    else if((length >= 10) && str_cmp(pBuffer, "AT+ADDR?", 8))
    {        
        sprintf(strTemp, "OK+LADD:%s\r\n", sys_config.mac_addr);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }    
    // 13、 连接最后一次连接成功的从设备
    else if((length == 10) && str_cmp(pBuffer, "AT+CONNL", 8))
    {
        /*
        Para: L, N, E,F
        L:连接中、N:空地址
        E:连接错误、F:连接失败
        */
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {        
            uint8 para[4] = {'L','N','E','F'};
            int8 id = 0;

            g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_CONNL;

            if(sys_config.ever_connect_peripheral_mac_addr_conut == 0)
            {
                id = 1;
            }
            else
            {
                id = 0;
                
                  // 开始扫描
                {
                    extern void simpleBLEStartScan();
                    simpleBLEStartScan();
                }
            }

            sprintf(strTemp, "AT+CONN%c\r\n", para[id]);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));    
        }
        else
        {
            ret = FALSE;
        }
    }
    // 14、连接指定蓝牙地址的主设备或从设备
    else if((length == 20) && str_cmp(pBuffer, "AT+CON", 6))
    {
        /*
        Para1: MAC地址、
        如: 0017EA0923AE
        Para2: A, E, F
        A: 连接中
        E: 连接错误
        F: 连接失败
        */
        uint8 para[3] = {'A','E','F'};
        uint8 id = 0;
        
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {
            g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_CON;
            osal_memcpy(sys_config.connect_mac_addr, pBuffer+6, MAC_ADDR_CHAR_LEN);

            sprintf(strTemp, "AT+CONN%c\r\n", para[id]);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            // 开始扫描
            {
                extern void simpleBLEStartScan();
                simpleBLEStartScan();
            }
        }
        else
        {
            ret = FALSE;
        }
    }
    // 15、 清除主设备配对信息
    else if((length == 10) && str_cmp(pBuffer, "AT+CLEAR", 8))
    {
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {        
            simpleBLE_SetAllParaDefault(PARA_PARI_FACTORY);
            //PrintAllPara();

            // 下面这个是 系统 API 函数， 删除所有配对信息
            GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
            
            sprintf(strTemp, "OK+CLEAR\r\n");
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {
            ret = FALSE;
        }
    }
    // 16、查询成功连接过的从机地址 
    // 备注：此指令只有在主设备时才有效；从设备时不接受此指令
    else if((length == 10) && str_cmp(pBuffer, "AT+RADD?", 8) 
        && (BLE_ROLE_CENTRAL == GetBleRole()))//仅主机有效
    {
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {        
            if(sys_config.ever_connect_peripheral_mac_addr_conut == 0)//无地址
            {
                sprintf(strTemp, "OK+RADD:NULL\r\n");
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }
            else
            {        
                for(i = 0; i<sys_config.ever_connect_peripheral_mac_addr_conut; i++)
                {
                    sprintf(strTemp, "OK+RADD:%s\r\n", sys_config.ever_connect_mac_status[i]);
                    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                }
            }
        }
        else
        {
            ret = FALSE;
        }
    }
    // 17、 查询软件版本
    else if((length == 10) && str_cmp(pBuffer, "AT+VERS?", 8))
    {
        sprintf(strTemp, "%s\r\n", VERSION);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // 18、 设置主模式下尝试连接时间
    else if((length == 10) && str_cmp(pBuffer, "AT+TCON", 7))
    {
        /*
        指令	                应答	            参数
        查询：AT+TCON?	        OK+TCON:[para] 	
        设置：AT+TCON[para]	    OK+Set:[para] 	    Para: 000000～009999 
                                                    000000 代表持续连接，其
                                                    余代表尝试的毫秒数
                                                    Default:001000
        */
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {        
            if(pBuffer[7] == '?')
            {
                sprintf(strTemp, "%06d\r\n", sys_config.try_connect_time_ms);
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }
            else
            {             
                sys_config.try_connect_time_ms = 10000;//_atoi(pBuffer+7);
                simpleBLE_WriteAllDataToFlash();
                sprintf(strTemp, "OK+Set:%06d\r\n", sys_config.try_connect_time_ms);
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }
        }
        else
        {
            ret = FALSE;
        }        
    }    
    // 19、 读取 RSSI 信号值
    else if((length == 10) && str_cmp(pBuffer, "AT+RSSI?", 10))
    {
        sprintf(strTemp, "OK+RSSI:%d\r\n", sys_config.rssi);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));        
    }
    // 20、  改变模块发射信号强度
    else if((length == 10) && str_cmp(pBuffer, "AT+TXPW", 7))
    {
        /*
        指令	        应答	            参数
        查询：          AT+TXPW?	        OK+ TXPW:[para]	
        设置：          AT+TXPW[para]	    OK+Set:[para]	Para: 0 ~ 3
                                            0: 4dbm、1: 0dbm
                                            2: -6dbm、3: -23dbm
                                            Default: 0
        */
        if(pBuffer[7] == '?')
        {
            sprintf(strTemp, "AT+TXPW:%d\r\n", sys_config.txPower);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {
            sys_config.txPower = pBuffer[7] - '0';
            if(sys_config.txPower > 3)
                sys_config.txPower = 0;
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.txPower);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

#if 1
        /*
#define LL_EXT_TX_POWER_MINUS_23_DBM                   0
#define LL_EXT_TX_POWER_MINUS_6_DBM                    1
#define LL_EXT_TX_POWER_0_DBM                          2
#define LL_EXT_TX_POWER_4_DBM                          3
        */
            // HCI_EXT_SetTxPowerCmd()是用来设置发射功率的. 有-23dbm, -6dbm, 0 dbm, +4dbm(仅CC2540)四个级别. 
            HCI_EXT_SetTxPowerCmd(3 - sys_config.txPower);
#endif      

            restart = TRUE;  //直接重启即可
         }
    }        
    // 21、  改变模块作为ibeacon基站广播时间间隔
    else if((length == 10 || length == 15) && str_cmp(pBuffer, "AT+TIBE", 7))
    {
        /*
        指令	        应答	        参数
        查询：          AT+TIBE?	    OK+ TIBE:[para]	
        设置：          AT+TIBE[para]	OK+Set:[para]	Para: 000000～009999 
                                        000000 代表持续广播，其
                                        余代表尝试的毫秒数
                                        Default:000500
        */
        if((GetBleRole() == BLE_ROLE_PERIPHERAL))
        {        
            if(pBuffer[7] == '?')
            {
                sprintf(strTemp, "AT+TIBE:%06d\r\n", sys_config.ibeacon_adver_time_ms);
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }
            else
            {   
                sys_config.ibeacon_adver_time_ms = str2Num(pBuffer+7, 6);
                if(sys_config.ibeacon_adver_time_ms <= 9999)
                {
                    simpleBLE_WriteAllDataToFlash();
                    sprintf(strTemp, "OK+Set:%06d\r\n", sys_config.ibeacon_adver_time_ms);
                    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

                    restart = TRUE;  //直接重启即可
                }
            }
        }
        else
        {
            ret = FALSE;
        }        
    }
    // 22、  设置模块工作类型 
    else if((length == 10) && str_cmp(pBuffer, "AT+IMME", 7))
    {
        /*
        指令	        应答	        参数
        查询：          AT+IMME?	    OK+Get:[para]	Para: 0~1
        设置：          AT+IMME[para]	OK+Set:[para]	Para: 0~1
                                        000000 代表持续广播，其
                                        0: 立即工作， 
                                        1: 等待AT+CON 或 AT+CONNL 命令
                                        Default:0
        */
        if(pBuffer[7] == '?')
        {
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.workMode);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {   
            sys_config.workMode = str2Num(pBuffer+7, 1);
            if(sys_config.workMode <= 1)
            {
                simpleBLE_WriteAllDataToFlash();
                sprintf(strTemp, "OK+Set:%d\r\n", sys_config.workMode);
                NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            }

            restart = TRUE;  //直接重启即可
        }
    }    
    // 23、 设置开始主模式下的从机扫描
    else if((length == 10) && str_cmp(pBuffer, "AT+DISC?\r\n", 10))
    {
        /*
        AT+DISC? OK+DISCS 无
        该指令为主模式、手动工作模式指令。使用前须满足 AT+ROLE1，AT+IMME1
        条件。
        发送 AT+DISC?指令后，模块若处于待机状态，返回 OK+DISCS 后开始搜索。
        如果搜到 BLE 设备会返回 OK+DISC:123456789012,其中数字字符串代表对
        方蓝牙地址，最多返回 6 个，下标从 0 开始。搜索完成后返回 OK+DISCE。
        如只搜到二个设备的例子：
        发送: AT+DISC?
        接收: OK+DISCS
        接收: OK+DISC:123456789012
        接收: OK+DISCE
        如果要连接搜到的设备，下标从 0 开始。
        发送: AT+CONN0 连接搜到的第一个设备
        发送: AT+CONN1 连接搜到的第二个设备
        依此类推
        */
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {
            g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_DISC;

            sprintf(strTemp, "OK+DISCS\r\n");
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            // 开始扫描
            {
                extern void simpleBLEStartScan();
                simpleBLEStartScan();
            }
        }
        else
        {
            ret = FALSE;
        }
    }
    // 24、 连接已经发现的从机， 输入的是下标号
    else if((length == 10) && str_cmp(pBuffer, "AT+CONN", 7))
    {
        /* 
        发送: AT+CONN0 连接搜到的第一个设备
        发送: AT+CONN1 连接搜到的第二个设备
        依此类推
        */
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {
            //sprintf(strTemp, "OK+DISCS\r\n");
            //NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_CONN;

            temp8 = pBuffer[7] - '0';
            // 开始扫描
            {
                extern bool simpleBLEConnect(uint8 index);
                simpleBLEConnect(temp8);
            }
        }
        else
        {
            ret = FALSE;
        }        
    }
    // 25、 AT+ADC 读取一路电压值读数
//    else if((length == 10) && str_cmp(pBuffer, "AT+SLEEP", 8))
//    {
//        g_sleepFlag = TRUE;
//        osal_pwrmgr_device( PWRMGR_BATTERY);   //  自动睡眠
//        //osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  不睡眠，功耗很高的        
//        // 格式化
//        sprintf(strTemp, "+SLEEP\r\nOK\r\n");
//        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
//        {
//            //设置p02的功能，1为uart脚， 0为输入中断脚
//            extern void HalKey_SET_P02_UART_RX(bool flag);

//            // 稍作延时 10ms                    
//            simpleBLE_Delay_1ms(10);
//            
//            HalKey_SET_P02_UART_RX(false);
//        }
//    }
    // 25、 AT+ADC 读取一路电压值读数
    else if((length == 9) && str_cmp(pBuffer, "AT+ADC", 6))
    {
        uint8 ch = pBuffer[6];  // AT+ADC的尾号，例如AT+ADC4中的4
        uint16 adc;         // adc 读数    
        float volt;         // 电压值
        uint8 channel;      // 通道
        
        switch(ch)
        {
        case '4'://测量P0.4端口电压 adc   
            channel = HAL_ADC_CHN_AIN4; goto NEXT_ADC;
        case '5'://测量P0.5端口电压 adc
            channel = HAL_ADC_CHN_AIN5; goto NEXT_ADC;
        case '6'://测量P0.6端口电压 adc
            channel = HAL_ADC_CHN_AIN6; goto NEXT_ADC;
        case '7'://测量P0.7端口电压 adc
            channel = HAL_ADC_CHN_AIN7; goto NEXT_ADC;
NEXT_ADC:
            HalAdcSetReference( HAL_ADC_REF_AVDD );
            adc = HalAdcRead( channel, HAL_ADC_RESOLUTION_14 ); //HAL_ADC_RESOLUTION_14  的有效位为13位，请看该函数内部处理
            volt = adc*3.3/8192;
            // 格式化
            sprintf(strTemp, "OK+Get:ADC%d=%d,%.2fv\r\n", ch-'0',adc, volt);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '8'://测量供电电压
        {
            // 参考自本协议栈中的 Battservice.c 文件
#define BATT_ADC_LEVEL_3V           409
#define BATT_ADC_LEVEL_2V           273
            uint16 battMinLevel = BATT_ADC_LEVEL_2V; // For VDD/3 measurements
            uint16 battMaxLevel = BATT_ADC_LEVEL_3V; // For VDD/3 measurements
            uint8 percent;

            HalAdcSetReference( HAL_ADC_REF_125V );
            adc = HalAdcRead( HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_10 );
            if (adc >= battMaxLevel)
            {
                percent = 100;
            }
            else if (adc <= battMinLevel)
            {
                percent = 0;
            }
            else
            {
                uint16 range =  battMaxLevel - battMinLevel + 1;                
                // optional if you want to keep it even, otherwise just take floor of divide
                // range += (range & 1);
                range >>= 2; // divide by 4
                // 百分比
                percent = (uint8) ((((adc - battMinLevel) * 25) + (range - 1)) / range);
            }
            // 格式化
            sprintf(strTemp, "OK+Get:ADC%d=%d,%d%%\r\n", ch-'0',adc, percent);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        }
        case '9'://测量芯片内部温度
        {
            /*
            参考网址
            https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/303585
            */
#define TEMP_CONST 0.61065 // (1250 / 2047) 
#define TEMP_OFFSET_DATASHEET 750 
#define TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS 29.75 
#define TEMP_OFFSET (TEMP_OFFSET_DATASHEET + TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS) // 779.75 
#define TEMP_COEFF 4.5  

            float AvgTemp;   
            float AvgTemp_sum = 0;               

            TR0 = 0x01;                     //设置为1来连接温度传感器到SOC_ADC
            ATEST = 0x01;                   //使能温度传感            

            // 求64个平均值，以便稳定
            for(i = 0; i<64; i++)
            {
                HalAdcSetReference( HAL_ADC_REF_125V );
                adc = HalAdcRead( HAL_ADC_CHN_TEMP, HAL_ADC_RESOLUTION_12 );
                AvgTemp = adc * TEMP_CONST; 
                AvgTemp =  ((AvgTemp - TEMP_OFFSET) / TEMP_COEFF); 
                AvgTemp_sum += AvgTemp;
            }
            // 得到实际温度
            AvgTemp = AvgTemp_sum/64; 
            
            TR0 &= ~0x01;
            ATEST &= ~0x01;
            
            // 格式化
            sprintf(strTemp, "OK+Get:ADC%d=%d,%.2f\r\n", ch-'0', adc, AvgTemp);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        }        
        
        default:    
            ret = FALSE;
            break;
        }        
    }
    else
    {
        ret = FALSE;
    }

    if(restart)//如果该标志已设置，稍微延时后重启
    {
        simpleBLE_Delay_1ms(200);      //设置参数后，适当延时， 以便上一次发送的数据正常发送出去
        HAL_SYSTEM_RESET(); 
    }
    
    return ret;
}
#endif

#if 1
/*
很多朋友问我们， 如何实现把主机或从机上的传感器数据直接发送到对端并通过主机的串口
透传出去， 下面我们就能实现这个功能， 不过到底需要什么样的传感器， 以及什么样的数据
就需要你自己来组织了， 下面这个函数每100ms执行一次:
都可以把数据发送到对端， 对端通过串口透传出去。
下面给出一个样例: 实现把字符串发送到对方
*/
void simpleBLE_SendMyData_ForTest()
{
    uint8 buffer[2] = {0};
    
    static uint16 count_100ms = 0;
    count_100ms++;
    
    if(count_100ms >= 600){//60s 
        
    DHT11();           //获取温湿度

    //将温湿度的转换成字符串
//    temp[0]=wendu_shi+0x30;
//    temp[1]=wendu_ge+0x30;
//    humidity[0]=shidu_shi+0x30;
//    humidity[1]=shidu_ge+0x30;
        
    //获得的温湿度通过串口输出到电脑显示
//    UartSendString(strTemp, 12);
//    UartSendString(temp, 2);
//    UartSendString("   ", 3);
//    
//    UartSendString(strHumidity, 9);
//    UartSendString(humidity, 2);
//    UartSendString("\n", 1);
    
    buffer[0]=wendu_shi;
    buffer[1]=shidu_shi;
    
    
    qq_write(buffer, 2);
    //qq_write(strTemp, 12);
    //qq_write(temp, 2);
    
    //qq_write(strHumidity, 9);
    //qq_write(humidity, 2);
    
    osal_set_event(simpleBLETaskId, SBP_DATA_EVT); 
    count_100ms=0;
    }
    
  
  
#if 0  
    static uint8 count_100ms = 0;
    uint8 numBytes;

    // 这里存在一些问题
    
    count_100ms++;
    if(count_100ms == 1)//本函数每100ms被执行一次， 计数10次就是1s
    {
        char strTemp[24] = {0};

        if((GetBleRole() == BLE_ROLE_CENTRAL) && simpleBLEChar6DoWrite && simpleBLECentralCanSend)               
        {
            sprintf(strTemp, "[%8ldms]Amo1\r\n", osal_GetSystemClock());
            //把你的数据组织到 strTemp， 就ok了, 注意不要超过 SIMPLEPROFILE_CHAR6_LEN 的大小
            //如果你发送的数据要超过 SIMPLEPROFILE_CHAR6_LEN， 那么最好的办法， 就是启动一个定时器，然后每定时器到， 就发送一段数据
            // 定时器的启动， 请参考       osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
            numBytes = (osal_strlen(strTemp) > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : osal_strlen(strTemp);            
            simpleBLE_UartDataMain((uint8*)strTemp, numBytes);
        }
        else if((GetBleRole() == BLE_ROLE_PERIPHERAL) && simpleBLEChar6DoWrite2)                    
        {
            sprintf(strTemp, "[%8ldms]Amo2\r\n", osal_GetSystemClock());
            //把你的数据组织到 strTemp， 就ok了, 注意不要超过 SIMPLEPROFILE_CHAR6_LEN 的大小
            //如果你发送的数据要超过 SIMPLEPROFILE_CHAR6_LEN， 那么最好的办法， 就是启动一个定时器，然后每定时器到， 就发送一段数据
            // 定时器的启动， 请参考       osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
            numBytes = (osal_strlen(strTemp) > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : osal_strlen(strTemp);
            simpleBLE_UartDataMain((uint8*)strTemp, numBytes);
        }

        count_100ms = 0;
    }
#endif    
}
#endif


static uint8 keyTestflag = ( HAL_KEY_SW_6 | HAL_KEY_UP | \
    HAL_KEY_RIGHT | HAL_KEY_CENTER \
    | HAL_KEY_LEFT | HAL_KEY_DOWN );     // 用于按键测试，出厂测试方便而已
/*
按键处理公共函数， 主机与从机都是运行这个函数，
注意每次启动不是主机就是从机，不是同时是主机与从机的， 所以他们不冲突的
*/
void simpleBLE_HandleKeys(uint8 keys)
{
    static char key_info[12] = {0};         // 保存按键信息
    static char key_info2[20] = {0};        // 测试专用，用于显示屏格式化比较好看
    bool key_pressed = true;                // 按键按下标记
    char str[24];                           // 缓冲区

    // 记录各种按键信息
    if ( keys & HAL_KEY_SW_6 )
    {
        keyTestflag &= (~HAL_KEY_SW_6);     // 表明该按键正常
        sprintf(key_info, "KEY_S1");
        sprintf(key_info2, "     KEY_S1");

        g_sleepFlag = !g_sleepFlag;   
        if(g_sleepFlag)
        {
            osal_pwrmgr_device( PWRMGR_BATTERY);   //  自动睡眠
        }
        else
        {
            osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  不睡眠，功耗很高的        
        }

        /* 睡眠标志
        打印 Sleep 0 表示没有睡眠，此时功耗较高，串口的rx线可以收到数据
        打印 Sleep 1 表示进入睡眠，此时功耗较低，串口的rx线不可以收到数据
        */
        sprintf(str, "Sleep %d ", g_sleepFlag);
        HalLcdWriteString(str, HAL_LCD_LINE_6);

        NPI_WriteTransport_ln((uint8*)str, strlen(str)); // 通过串口发送出来
        simpleBLE_Delay_1ms(1);
    }
#if defined ( AMOMCU_UART_RX_MODE)
    else if ( keys & HAL_KEY_SW_7 )
    {        
        sprintf(key_info, "KEY_S7");

        {
            //设置p02的功能，1为uart脚， 0为输入中断脚
            HalKey_Set_P02_for_UartRX_or_GPIO(true);
            //osal_pwrmgr_device( PWRMGR_BATTERY);   //  自动睡眠
            osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  不睡眠，功耗很高的
            osal_start_timerEx( simpleBLETaskId, SBP_WAKE_EVT, 10);// 10ms后数据才有效， 这个中间的10，留给串口接收唤醒串口的数据， 该数据视为无效
        }        
    }
#endif
    else if ( keys & HAL_KEY_UP )
    {
        keyTestflag &= (~HAL_KEY_UP);     // 表明该按键正常

        sprintf(key_info, "KEY_UP");
        sprintf(key_info2, "  KEY_UP");


        if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
        {
            if(simpleBLEState != BLE_STATE_CONNECTED)
            {   // 搜索从设备
                simpleBLE_AT_CMD_Handle("AT+DISC?\r\n", 10);
            }
        }
    }
    else if ( keys & HAL_KEY_DOWN )
    {
        keyTestflag &= (~HAL_KEY_DOWN);     // 表明该按键正常

        sprintf(key_info, "KEY_DOWN");
        sprintf(key_info2, "      KEY_DOWN");

        // 打开或者关闭rssi测距功能
        g_rssi_flag = !g_rssi_flag;
    }  
    else if ( keys & HAL_KEY_LEFT )
    {
        keyTestflag &= (~HAL_KEY_LEFT);     // 表明该按键正常

        sprintf(key_info, "KEY_LEFT");
        sprintf(key_info2, "KEY_LEFT");

        if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
        {
            if(simpleBLEState != BLE_STATE_CONNECTED)
            {   // 显示下一个从设备 
                // 显示下一个从设备的地址  nextFalg=true则显示下一个地址，否则显示当前地址
                extern void simpleBLECentraDisplaNextPeriAddr(bool nextFalg);
                simpleBLECentraDisplaNextPeriAddr(TRUE);   
            }
        }
    }  
    else if ( keys & HAL_KEY_RIGHT )
    {
        keyTestflag &= (~HAL_KEY_RIGHT);     // 表明该按键正常

        sprintf(key_info, "KEY_RIGHT");
        sprintf(key_info2, "        KEY_RIGHT");
    }  
    else if ( keys & HAL_KEY_CENTER )
    {
        char buffer[12];

        keyTestflag &= (~HAL_KEY_CENTER);     // 表明该按键正常

        sprintf(key_info, "KEY_CENTER");
        sprintf(key_info2, "    KEY_CENTER");

        if(GetBleRole() == BLE_ROLE_CENTRAL)//主机
        {
            if(simpleBLEState != BLE_STATE_CONNECTED)
            {   // 连接当前界面上显示的mac地址的从设备， 这里是索引号
                extern uint8 simpleBLECentraGetAddrId();
                uint8 id = simpleBLECentraGetAddrId();
                sprintf(buffer, "AT+CONN%d\r\n", id);
                simpleBLE_AT_CMD_Handle((uint8*)buffer, strlen(buffer));
            }
            else if(simpleBLE_IfConnected()) 
            {
                // 测试代码，发送一个指令， 灯亮或熄灭从设备的LED2
                static uint8 led2OnOff = 0;
                led2OnOff = led2OnOff ? 0 : 1;
                
                sprintf(buffer, "AT+LED2=%d\r\n", led2OnOff);
                qq_write((uint8*)buffer, strlen(buffer));

                // 启动事件，然后在事件中再启动定时器定时检测数据并发送到网络
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );
            }
        } 
        else // 从设备，下面发送点灯指令点亮主机的 LED3
        {
            if(simpleBLE_IfConnected()) 
            {
                // 测试代码，发送一个指令， 灯亮或熄灭主设备的LED3
                static uint8 led2OnOff = 0;
                led2OnOff = led2OnOff ? 0 : 1;
                
                sprintf(buffer, "AT+LED3=%d\r\n", led2OnOff);
                qq_write((uint8*)buffer, strlen(buffer));

                // 启动事件，然后在事件中再启动定时器定时检测数据并发送到网络
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );
            }
        }
    }
    else
    {
        key_pressed = false;
    }

    if(1)  // 判断所有按键都已经被按下过，则一次性点亮3个led灯
    {
        static bool keyTestFisrtFinish = false;
        if(!keyTestflag && !keyTestFisrtFinish)
        {
            keyTestFisrtFinish = true;
            HalLedSet(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, HAL_LED_MODE_ON);
        }
    }
    
    // 按键按下
    if(key_pressed)
    {   
        // 显示按键信息，如果全部按键都按下过，表示测试按键成功，反色显示
        sprintf(str, "%s ON", key_info2);
        HalLcdWriteString(str, HAL_LCD_LINE_8 | (keyTestflag ? 0 : HAL_LCD_REVERSE));

        sprintf(str, "%s ON", key_info);
    }
    else
    {
        // 显示按键信息，如果全部按键都按下过，表示测试按键成功，反色显示
        sprintf(str, "%s OFF", key_info2);
        HalLcdWriteString(str, HAL_LCD_LINE_8 | (keyTestflag ? 0 : HAL_LCD_REVERSE));

        sprintf(str, "%s OFF", key_info);
    }

    /*
    发送按键信息:
    如果未连接上主机或者从机，则通过串口发送出来
    如果已连接上主机或者从机，则把信息发送到对端
    */
    if(1)
    {
        char buffer[20];
        sprintf(buffer, "%s\r\n", str);
        
        if(!simpleBLE_IfConnected())// 未连接
        {
            NPI_WriteTransport((uint8*)buffer, strlen(buffer)); // 通过串口发送出来
            simpleBLE_Delay_1ms(1);
        }
        else //已连接
        {         
            qq_write((uint8*)buffer, strlen(buffer));

            // 启动事件，然后在事件中再启动定时器定时检测数据并发送到网络
            osal_set_event( simpleBLETaskId, SBP_UART_EVT );                    
        }
    }
}


// MT 命令处理 函数
bool simpleBLE_MT_CMD_Handle(uint8 *pBuffer, uint16 length)
{
    char strTemp[64];
    uint8 i;

    /*
    支持命令如下:
    AT+LED2=0  灭LED2灯
    AT+LED2=1  亮LED2灯
    AT+LED3=0  灭LED3灯
    AT+LED3=1  亮LED3灯
    由于LED1作为系统指示灯，这里就不作为控制来用了， 其他的请自行添加
    */
    uint8 onoff;
    if( (length >= 9) && str_cmp(pBuffer, "AT+LED2=", 8))
    {
        onoff = pBuffer[8];
        if(onoff == '0')
        {
            HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
        }
        else
        {
            HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
        }
    }
    else if( (length >= 9) && str_cmp(pBuffer, "AT+LED3=", 8))
    {
        onoff = pBuffer[8];
        if(onoff == '0')
        {
            HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF);
        }
        else
        {
            HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
        }
    }
    else if( (length >= 9) && str_cmp(pBuffer, "AT+RSSI=", 8))
    {
        onoff = pBuffer[8];
        if(onoff == '0')
        {
            g_rssi_flag = 0;
        }
        else
        {
            g_rssi_flag = 1;
        }
    }
    
    else if((length >= 7) && str_cmp(pBuffer, "AT+ADC", 6))
    {
        uint8 ch = pBuffer[6];  // AT+ADC的尾号，例如AT+ADC4中的4
        uint16 adc;         // adc 读数    
        float volt;         // 电压值
        uint8 channel;      // 通道
        
        switch(ch)
        {
            case '4'://测量P0.4端口电压 adc   
                channel = HAL_ADC_CHN_AIN4; goto NEXT_ADC;
            case '5'://测量P0.5端口电压 adc
                channel = HAL_ADC_CHN_AIN5; goto NEXT_ADC;
            case '6'://测量P0.6端口电压 adc
                channel = HAL_ADC_CHN_AIN6; goto NEXT_ADC;
            case '7'://测量P0.7端口电压 adc
                channel = HAL_ADC_CHN_AIN7; goto NEXT_ADC;
    NEXT_ADC:
                HalAdcSetReference( HAL_ADC_REF_AVDD );
                adc = HalAdcRead( channel, HAL_ADC_RESOLUTION_14 ); //HAL_ADC_RESOLUTION_14  的有效位为13位，请看该函数内部处理
                volt = adc*3.3/8192;
                // 格式化
                sprintf(strTemp, "OK+Get:ADC%d=%d,%.2fv\r\n", ch-'0',adc, volt);
    //            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                qq_write((uint8*)strTemp, strlen(strTemp));
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );                 
                break;
            case '8'://测量供电电压
            {
                // 参考自本协议栈中的 Battservice.c 文件
#define BATT_ADC_LEVEL_3V           409
#define BATT_ADC_LEVEL_2V           273
                uint16 battMinLevel = BATT_ADC_LEVEL_2V; // For VDD/3 measurements
                uint16 battMaxLevel = BATT_ADC_LEVEL_3V; // For VDD/3 measurements
                uint8 percent;

                HalAdcSetReference( HAL_ADC_REF_125V );
                adc = HalAdcRead( HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_10 );
                if (adc >= battMaxLevel)
                {
                    percent = 100;
                }
                else if (adc <= battMinLevel)
                {
                    percent = 0;
                }
                else
                {
                    uint16 range =  battMaxLevel - battMinLevel + 1;                
                    // optional if you want to keep it even, otherwise just take floor of divide
                    // range += (range & 1);
                    range >>= 2; // divide by 4
                    // 百分比
                    percent = (uint8) ((((adc - battMinLevel) * 25) + (range - 1)) / range);
                }
                // 格式化
                sprintf(strTemp, "OK+Get:ADC%d=%d,%d%%\r\n", ch-'0',adc, percent);
    //            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                qq_write((uint8*)strTemp, strlen(strTemp));
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );                 
                break;
            }
            case '9'://测量芯片内部温度
            {
                /*
                参考网址
                https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/303585
                */
#define TEMP_CONST 0.61065 // (1250 / 2047) 
#define TEMP_OFFSET_DATASHEET 750 
#define TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS 29.75 
#define TEMP_OFFSET (TEMP_OFFSET_DATASHEET + TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS) // 779.75 
#define TEMP_COEFF 4.5  

                float AvgTemp;   
                float AvgTemp_sum = 0;               

                TR0 = 0x01;                     //设置为1来连接温度传感器到SOC_ADC
                ATEST = 0x01;                   //使能温度传感            

                // 求64个平均值，以便稳定
                for(i = 0; i<64; i++)
                {
                    HalAdcSetReference( HAL_ADC_REF_125V );
                    adc = HalAdcRead( HAL_ADC_CHN_TEMP, HAL_ADC_RESOLUTION_12 );
                    AvgTemp = adc * TEMP_CONST; 
                    AvgTemp =  ((AvgTemp - TEMP_OFFSET) / TEMP_COEFF); 
                    AvgTemp_sum += AvgTemp;
                }
                // 得到实际温度
                AvgTemp = AvgTemp_sum/64; 
                
                TR0 &= ~0x01;
                ATEST &= ~0x01;
                
                // 格式化
                sprintf(strTemp, "OK+Get:ADC%d=%d,%.2f\r\n", ch-'0', adc, AvgTemp);
    //            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                qq_write((uint8*)strTemp, strlen(strTemp));
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );                 
                break;
            }            
        }   
    }

    return TRUE;
}

#if defined(USE_DISPLAY_KEY_VALUE)  // 测试按键专用，显示5向按键值
void SimpleBLE_DisplayTestKeyValue()
{
    uint16 adc;
    uint8 ksave0 = 0;
    char str1[24]={0};
    char str2[14]={0};
    float volt = 0.0;    

    if(keyTestflag)
    {
        HalAdcSetReference( HAL_ADC_REF_AVDD );    
        P0SEL &= ~0x40; // 配置P0.6为功能脚，否则实测是会发现管脚有波形跳变
        P0DIR &= ~0x40;
        adc = HalAdcRead (HAL_ADC_CHANNEL_6, HAL_ADC_RESOLUTION_8); //仅返回7位数据，最大值128-1
        //adc = 100;
        if ((adc >= 2) && (adc <= 38))
        {
           ksave0 |= HAL_KEY_UP;
           sprintf(str1, "key:(%2d,%3d)", 2, 38);       
        }
        else if ((adc >= 74) && (adc <= 88))
        {
          ksave0 |= HAL_KEY_RIGHT;
          sprintf(str1, "key:(%2d,%3d)", 74, 88);       
        }
        else if ((adc >= 60) && (adc <= 73))
        {
          ksave0 |= HAL_KEY_LEFT;
          sprintf(str1, "key:(%2d,%3d)", 60, 73);       
        }
        else if ((adc >= 39) && (adc <= 59))
        {
          ksave0 |= HAL_KEY_DOWN;
          sprintf(str1, "key:(%2d,%3d)", 39, 59);       
        }
        else if ((adc >= 89) && (adc <= 100))
        {
          ksave0 |= HAL_KEY_CENTER;
          sprintf(str1, "key:(%2d,%3d)", 89, 100);       
        }
        else
        {
          ksave0 |= 0;
          sprintf(str1, "key:(ERROR)");       
        }        

        volt = (float)adc*3.3/128;
        sprintf(str2, " %d=%0.1fv", adc, volt);
        strcat(str1,str2);
        HalLcdWriteString(str1, HAL_LCD_LINE_7);    
    }
    else
    {
        HalLcdWriteString("    KEY PASS !!!  ", HAL_LCD_LINE_7 | HAL_LCD_REVERSE);    
    }
}
#endif
