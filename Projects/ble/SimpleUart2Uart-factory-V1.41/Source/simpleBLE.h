#ifndef SIMPLEBLE_H
#define SIMPLEBLE_H

#ifdef __cplusplus
extern "C"
{
#endif


// 主机从机事件共用定义
#define START_DEVICE_EVT                               0x0001//启动设备
#define SBP_PERIODIC_EVT                                0x0002//系统轮询定时器
#define SBP_DATA_EVT                                   0x0004//数据传输
#define SBP_VABRATIVE_EVT                              0x0008//震动器中断事件
#define SBP_UART_EVT                                   0x0010//串口数据事件
#define SBP_SLEEP_EVT                                  0x0020//睡眠事件
#define SBP_WAKE_EVT                                   0x0040//唤醒事件
#define SBP_PERIODIC_LED_EVT                          0x0080//从设备专用-led闪缩专用
#define SBP_CONNECT_EVT                                0x0100//主机连接-确认连接
#define START_DISCOVERY_EVT                            0x0200//发现从设备



#define SLEEP_MS                                        300  //睡眠时间， 串口激活事件


//------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

//#define RELEASE_VER                      //定义版本发布用
#define     VERSION     "v2.0"  // for etoh, 2015/9/9

//设备名称的字符长度 <= 12
#define DEV_NAME_DEFAULT                           "AIWAC_DIAPERS"//"Amomcu"


//设备名称的字符长度 <= 12
#define DEV_NAME_LEN                                12


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//以下参数请勿修改--------------------------------amomcu.com--------------------


// 系统定时器间隔时间
#define SBP_PERIODIC_EVT_PERIOD                   100//必须是100ms

//最大记录的从机地址
#define MAX_PERIPHERAL_MAC_ADDR                   10//最大记录的从机地址

//mac地址的字符长度 (一个字节等于两个字符)
#define MAC_ADDR_CHAR_LEN                          12//mac地址的字符长度 (一个字节等于两个字符)


// 出厂设置或清除配对信息与从机信息
typedef enum
{
    PARA_ALL_FACTORY = 0,           //全部恢复出厂设置
    PARA_PARI_FACTORY = 1,          //配对信息恢复出厂设置-相当于清除配对信息与从机信息
}PARA_SET_FACTORY;

// 当前单片机运行的角色
typedef enum
{
    BLE_ROLE_PERIPHERAL = 0,        //从机角色
    BLE_ROLE_CENTRAL = 1,           //主机角色    
}BLE_ROLE;

// 应用程序状态
enum
{
  BLE_STATE_IDLE,                    //无连接-空闲状态
  BLE_STATE_CONNECTING,             //连接中...
  BLE_STATE_CONNECTED,              //已连接上
  BLE_STATE_DISCONNECTING,          //断开连接中
  BLE_STATE_ADVERTISING             //从机广播中
};

// 系统裕兴模式定义
enum
{
  BLE_MODE_SERIAL,                   // 串口透传模式 【默认】
  BLE_MODE_DRIVER,                   // 直驱模式        
  BLE_MODE_iBeacon,                  // iBeacon 广播模式
  BLE_MODE_MAX,
};

// 连接模式指示
typedef enum
{
  CONNECT_MODE_FIX_ADDR_CONNECTED,     // 指定 mac地址进行连接
  CONNECT_MODE_LAST_ADDR_CONNECTED,    // 连接最后成功连接过的 mac地址
  CONNECT_MODE_MAX,
}CONNECT_MODE;


// 应用程序状态
typedef enum
{
  BLE_CENTRAL_CONNECT_CMD_NULL,              //主机 AT 连接命令  空
  BLE_CENTRAL_CONNECT_CMD_CONNL,             //主机 AT 连接命令  连接最近成功过的地址
  BLE_CENTRAL_CONNECT_CMD_CON,               //主机 AT 连接命令  连接指定地址
  BLE_CENTRAL_CONNECT_CMD_DISC,              //主机 AT 扫描从机命令
  BLE_CENTRAL_CONNECT_CMD_CONN,              //主机 AT 连接命令  连接扫描到的地址的下标号对应的地址
}BLE_CENTRAL_CONNECT_CMD;
extern BLE_CENTRAL_CONNECT_CMD g_Central_connect_cmd ;


// 定于系统结构变量， 该结构会在开机时从nv flash 中读取， 数据有修改时， 需要写入nv flash
// 这样， 就实现了系统重启后数据还是上一次设置的
typedef struct 
{
    /*
    波特率
    0---------9600 
    1---------19200 
    2---------38400 
    3---------57600 
    4---------115200
    */
    uint8 baudrate;                 //波特率 ， 目前支持的列表如上
    uint8 parity;                   //校验位    
    uint8 stopbit;                  //停止位
    
    uint8 mode;                     //工作模式 0:透传 ， 1: 直驱 , 2: iBeacon

    // 设备名称，最长 11 位数字或字母，含中划线和下划线，不建议用其它字符    
    uint8 name[12];                 

    BLE_ROLE role;                  //主从模式  0: 从机   1: 主机

    uint8 pass[7];                  //密码， 最大6位 000000~999999 

    /*
    Para: 0 ~ 1 
    0: 连接不需要密码
    1: 连接需要密码
    */
    uint8 type;                     //鉴权模式

    
    uint8 mac_addr[MAC_ADDR_CHAR_LEN+1];            //本机mac地址 最大12位 字符表示
    uint8 connect_mac_addr[MAC_ADDR_CHAR_LEN+1];    //指定去连接的mac地址


    //曾经成功连接过的从机个数
    uint8 ever_connect_peripheral_mac_addr_conut;
    //曾经成功连接过的从机个数,当前index， 用于增加从机地址时快速插入或读取
    uint8 ever_connect_peripheral_mac_addr_index;
    //最新一次成功连接过的从机地址index， 用于针对AT+CONNL 这个指令
    uint8 last_connect_peripheral_mac_addr_index;
    //曾经成功连接过的从机地址
    uint8 ever_connect_mac_status[MAX_PERIPHERAL_MAC_ADDR][MAC_ADDR_CHAR_LEN];       

    /*
    Para: 000000～009999 
    000000 代表持续连接，其
    余代表尝试的毫秒数
    Default:001000
    */
    
    uint16 try_connect_time_ms;           // 尝试连接时间---目前无效
    int8 rssi;                              //  RSSI 信号值
    uint8 rxGain;                           //  接收增益强度
    uint8 txPower;                          //  发射信号强度
    uint16 ibeacon_adver_time_ms;         // 广播间隔

    //  模块工作类型  0: 立即工作， 1: 等待AT+CON 或 AT+CONNL 命令
    uint8 workMode;                        
}SYS_CONFIG;
extern SYS_CONFIG sys_config;



extern uint8 simpleBLEState;
extern uint16 simpleBLECharHdl;
extern uint16 simpleBLECharHd6;
extern bool simpleBLEChar6DoWrite;
extern bool simpleBLEChar6DoWrite2;
extern bool simpleBLEChar7DoWrite;


#if defined (RELEASE_VER)
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)

#if defined (HAL_LCD)
#undef HAL_LCD
#define HAL_LCD FALSE 
#endif

#else
// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif
#endif





extern uint8 simpleBLETaskId;               // 主机任务
extern uint8 simpleBLEState;
extern uint16 simpleBLECharHdl;
extern uint16 simpleBLECharHd6;
extern bool simpleBLECentralCanSend;
extern bool simpleBLEChar6DoWrite;
extern uint8 simpleBLEPeripheral_TaskID;        // 从机任务



#if 1
// 该函数延时时间为1ms， 用示波器测量过， 稍有误差， 但误差很小  --amomcu.com
void simpleBLE_Delay_1ms(int times);

// 字符串对比
uint8 str_cmp(uint8 *p1,uint8 *p2,uint8 len);

// 字符串转数字
uint32 str2Num(uint8* numStr, uint8 iLength);

char *bdAddr2Str( uint8 *pAddr );

// 保存所有数据到nv flash
void simpleBLE_WriteAllDataToFlash();

// 读取自定义的 nv flash 数据  -------未使用到
void simpleBLE_ReadAllDataToFlash();

//flag: PARA_ALL_FACTORY:  全部恢复出厂设置
//flag: PARA_PARI_FACTORY: 清除配对信息
void simpleBLE_SetAllParaDefault(PARA_SET_FACTORY flag); 

// 打印所有存储到nv flash的数据， 方便调试代码
void PrintAllPara(void);


// 返回设备角色
//主从模式  0: 从机   1: 主机
BLE_ROLE GetBleRole();


// 判断蓝牙是否连接上
// 0: 未连接上
// 1: 已连接上
bool simpleBLE_IfConnected();

// 增加从机地址， 注意， 需要连接成功后， 再增加该地址
void simpleBLE_SetPeripheralMacAddr(uint8 *pAddr);


// 读取从机地址, index < MAX_PERIPHERAL_MAC_ADDR
// 用于判断是否系统中已存有该Mac地址
/*
index: 应该是 < MAX_PERIPHERAL_MAC_ADDR,
*/
bool simpleBLE_GetPeripheralMacAddr(uint8 index, uint8 *pAddr);


// 有按键按下，则启动为主机， 否则默认启动为从机
// 0 启动peripheral从设备， 1: 启动为 central
bool Check_startup_peripheral_or_central(void);


//开机时判断到按键按下3秒， 恢复出厂设置
//按键定义为  p0.7
void CheckKeyForSetAllParaDefault(void); 

// 串行口 uart 初始化
void simpleBLE_NPI_init(void);

// 设置接收增益
void UpdateRxGain(void);

// 设置发射功率
void UpdateTxPower(void);

// 设置led灯的状态
void simpleBle_LedSetState(uint8 onoff);

// 保存RSSI 到系统变量
void simpleBle_SetRssi(int8 rssi);

// 串口打印密码  -----测试用----
void simpleBle_PrintPassword();

// 获取设备名称
uint8* simpleBle_GetAttDeviceName();

// 主机是否记录了从机地址
bool simpleBle_IFfHavePeripheralMacAddr( void );

// 定时器任务定时执行函数， 用于设置led的状态----也可以增加一个定时器来做
void simpleBLE_performPeriodicTask( void );

// 获取鉴权要求, 0: 连接不需要密码,  1: 连接需要密码
bool simpleBle_GetIfNeedPassword();

// 获取连接密码
uint32 simpleBle_GetPassword();

// 判断是否是 iBeacon 广播模式
bool simpleBLE_CheckIfUse_iBeacon();

// 判断是否使能串口透传
bool simpleBLE_CheckIfUse_Uart2Uart();

// 判断是输入的形参-地址是否是需要去连接的地址，如果是， 返回真， 否则返回假
bool simpleBLE_GetToConnectFlag(uint8 *Addr);

// 设置 iBeacon 的广播间隔
uint32 simpleBLE_GetiBeaconAdvertisingInterral();

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
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events );


// AT 命令处理 函数
bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length);
// MT 命令处理 函数
bool simpleBLE_MT_CMD_Handle(uint8 *pBuffer, uint16 length);


/*
很多朋友问我们， 如何实现把主机或从机上的传感器数据直接发送到对端并通过主机的串口
透传出去， 下面我们就能实现这个功能， 不过到底需要什么样的传感器， 以及什么样的数据
就需要你自己来组织了， 下面这个函数每100ms执行一次:
都可以把数据发送到对端， 对端通过串口透传出去。
下面给出一个样例: 实现把字符串发送到对方
*/
void simpleBLE_SendMyData_ForTest();


/*
按键处理公共函数， 主机与从机都是运行这个函数，
注意每次启动不是主机就是从机，不是同时是主机与从机的， 所以他们不冲突的
*/
void simpleBLE_HandleKeys(uint8 keys);

#if defined(USE_DISPLAY_KEY_VALUE)  // 测试按键专用，显示5向按键值
void SimpleBLE_DisplayTestKeyValue();
#endif

extern bool g_sleepFlag;    //睡眠标志
extern uint8 uart_sleep_count; // 睡眠计数器
extern bool g_rssi_flag;       //是否开启测距


#endif



#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLE_H */
