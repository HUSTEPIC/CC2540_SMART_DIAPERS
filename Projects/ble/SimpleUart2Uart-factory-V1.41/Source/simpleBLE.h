#ifndef SIMPLEBLE_H
#define SIMPLEBLE_H

#ifdef __cplusplus
extern "C"
{
#endif


// �����ӻ��¼����ö���
#define START_DEVICE_EVT                               0x0001//�����豸
#define SBP_PERIODIC_EVT                                0x0002//ϵͳ��ѯ��ʱ��
#define SBP_DATA_EVT                                   0x0004//���ݴ���
#define SBP_VABRATIVE_EVT                              0x0008//�����ж��¼�
#define SBP_UART_EVT                                   0x0010//���������¼�
#define SBP_SLEEP_EVT                                  0x0020//˯���¼�
#define SBP_WAKE_EVT                                   0x0040//�����¼�
#define SBP_PERIODIC_LED_EVT                          0x0080//���豸ר��-led����ר��
#define SBP_CONNECT_EVT                                0x0100//��������-ȷ������
#define START_DISCOVERY_EVT                            0x0200//���ִ��豸



#define SLEEP_MS                                        300  //˯��ʱ�䣬 ���ڼ����¼�


//------------------------------------------------------------------------------
//--------------------------------------------------------------------------------

//#define RELEASE_VER                      //����汾������
#define     VERSION     "v2.0"  // for etoh, 2015/9/9

//�豸���Ƶ��ַ����� <= 12
#define DEV_NAME_DEFAULT                           "AIWAC_DIAPERS"//"Amomcu"


//�豸���Ƶ��ַ����� <= 12
#define DEV_NAME_LEN                                12


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//���²��������޸�--------------------------------amomcu.com--------------------


// ϵͳ��ʱ�����ʱ��
#define SBP_PERIODIC_EVT_PERIOD                   100//������100ms

//����¼�Ĵӻ���ַ
#define MAX_PERIPHERAL_MAC_ADDR                   10//����¼�Ĵӻ���ַ

//mac��ַ���ַ����� (һ���ֽڵ��������ַ�)
#define MAC_ADDR_CHAR_LEN                          12//mac��ַ���ַ����� (һ���ֽڵ��������ַ�)


// �������û���������Ϣ��ӻ���Ϣ
typedef enum
{
    PARA_ALL_FACTORY = 0,           //ȫ���ָ���������
    PARA_PARI_FACTORY = 1,          //�����Ϣ�ָ���������-�൱����������Ϣ��ӻ���Ϣ
}PARA_SET_FACTORY;

// ��ǰ��Ƭ�����еĽ�ɫ
typedef enum
{
    BLE_ROLE_PERIPHERAL = 0,        //�ӻ���ɫ
    BLE_ROLE_CENTRAL = 1,           //������ɫ    
}BLE_ROLE;

// Ӧ�ó���״̬
enum
{
  BLE_STATE_IDLE,                    //������-����״̬
  BLE_STATE_CONNECTING,             //������...
  BLE_STATE_CONNECTED,              //��������
  BLE_STATE_DISCONNECTING,          //�Ͽ�������
  BLE_STATE_ADVERTISING             //�ӻ��㲥��
};

// ϵͳԣ��ģʽ����
enum
{
  BLE_MODE_SERIAL,                   // ����͸��ģʽ ��Ĭ�ϡ�
  BLE_MODE_DRIVER,                   // ֱ��ģʽ        
  BLE_MODE_iBeacon,                  // iBeacon �㲥ģʽ
  BLE_MODE_MAX,
};

// ����ģʽָʾ
typedef enum
{
  CONNECT_MODE_FIX_ADDR_CONNECTED,     // ָ�� mac��ַ��������
  CONNECT_MODE_LAST_ADDR_CONNECTED,    // �������ɹ����ӹ��� mac��ַ
  CONNECT_MODE_MAX,
}CONNECT_MODE;


// Ӧ�ó���״̬
typedef enum
{
  BLE_CENTRAL_CONNECT_CMD_NULL,              //���� AT ��������  ��
  BLE_CENTRAL_CONNECT_CMD_CONNL,             //���� AT ��������  ��������ɹ����ĵ�ַ
  BLE_CENTRAL_CONNECT_CMD_CON,               //���� AT ��������  ����ָ����ַ
  BLE_CENTRAL_CONNECT_CMD_DISC,              //���� AT ɨ��ӻ�����
  BLE_CENTRAL_CONNECT_CMD_CONN,              //���� AT ��������  ����ɨ�赽�ĵ�ַ���±�Ŷ�Ӧ�ĵ�ַ
}BLE_CENTRAL_CONNECT_CMD;
extern BLE_CENTRAL_CONNECT_CMD g_Central_connect_cmd ;


// ����ϵͳ�ṹ������ �ýṹ���ڿ���ʱ��nv flash �ж�ȡ�� �������޸�ʱ�� ��Ҫд��nv flash
// ������ ��ʵ����ϵͳ���������ݻ�����һ�����õ�
typedef struct 
{
    /*
    ������
    0---------9600 
    1---------19200 
    2---------38400 
    3---------57600 
    4---------115200
    */
    uint8 baudrate;                 //������ �� Ŀǰ֧�ֵ��б�����
    uint8 parity;                   //У��λ    
    uint8 stopbit;                  //ֹͣλ
    
    uint8 mode;                     //����ģʽ 0:͸�� �� 1: ֱ�� , 2: iBeacon

    // �豸���ƣ�� 11 λ���ֻ���ĸ�����л��ߺ��»��ߣ��������������ַ�    
    uint8 name[12];                 

    BLE_ROLE role;                  //����ģʽ  0: �ӻ�   1: ����

    uint8 pass[7];                  //���룬 ���6λ 000000~999999 

    /*
    Para: 0 ~ 1 
    0: ���Ӳ���Ҫ����
    1: ������Ҫ����
    */
    uint8 type;                     //��Ȩģʽ

    
    uint8 mac_addr[MAC_ADDR_CHAR_LEN+1];            //����mac��ַ ���12λ �ַ���ʾ
    uint8 connect_mac_addr[MAC_ADDR_CHAR_LEN+1];    //ָ��ȥ���ӵ�mac��ַ


    //�����ɹ����ӹ��Ĵӻ�����
    uint8 ever_connect_peripheral_mac_addr_conut;
    //�����ɹ����ӹ��Ĵӻ�����,��ǰindex�� �������Ӵӻ���ַʱ���ٲ�����ȡ
    uint8 ever_connect_peripheral_mac_addr_index;
    //����һ�γɹ����ӹ��Ĵӻ���ַindex�� �������AT+CONNL ���ָ��
    uint8 last_connect_peripheral_mac_addr_index;
    //�����ɹ����ӹ��Ĵӻ���ַ
    uint8 ever_connect_mac_status[MAX_PERIPHERAL_MAC_ADDR][MAC_ADDR_CHAR_LEN];       

    /*
    Para: 000000��009999 
    000000 ����������ӣ���
    ������Եĺ�����
    Default:001000
    */
    
    uint16 try_connect_time_ms;           // ��������ʱ��---Ŀǰ��Ч
    int8 rssi;                              //  RSSI �ź�ֵ
    uint8 rxGain;                           //  ��������ǿ��
    uint8 txPower;                          //  �����ź�ǿ��
    uint16 ibeacon_adver_time_ms;         // �㲥���

    //  ģ�鹤������  0: ���������� 1: �ȴ�AT+CON �� AT+CONNL ����
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





extern uint8 simpleBLETaskId;               // ��������
extern uint8 simpleBLEState;
extern uint16 simpleBLECharHdl;
extern uint16 simpleBLECharHd6;
extern bool simpleBLECentralCanSend;
extern bool simpleBLEChar6DoWrite;
extern uint8 simpleBLEPeripheral_TaskID;        // �ӻ�����



#if 1
// �ú�����ʱʱ��Ϊ1ms�� ��ʾ������������ ������ ������С  --amomcu.com
void simpleBLE_Delay_1ms(int times);

// �ַ����Ա�
uint8 str_cmp(uint8 *p1,uint8 *p2,uint8 len);

// �ַ���ת����
uint32 str2Num(uint8* numStr, uint8 iLength);

char *bdAddr2Str( uint8 *pAddr );

// �����������ݵ�nv flash
void simpleBLE_WriteAllDataToFlash();

// ��ȡ�Զ���� nv flash ����  -------δʹ�õ�
void simpleBLE_ReadAllDataToFlash();

//flag: PARA_ALL_FACTORY:  ȫ���ָ���������
//flag: PARA_PARI_FACTORY: ��������Ϣ
void simpleBLE_SetAllParaDefault(PARA_SET_FACTORY flag); 

// ��ӡ���д洢��nv flash�����ݣ� ������Դ���
void PrintAllPara(void);


// �����豸��ɫ
//����ģʽ  0: �ӻ�   1: ����
BLE_ROLE GetBleRole();


// �ж������Ƿ�������
// 0: δ������
// 1: ��������
bool simpleBLE_IfConnected();

// ���Ӵӻ���ַ�� ע�⣬ ��Ҫ���ӳɹ��� �����Ӹõ�ַ
void simpleBLE_SetPeripheralMacAddr(uint8 *pAddr);


// ��ȡ�ӻ���ַ, index < MAX_PERIPHERAL_MAC_ADDR
// �����ж��Ƿ�ϵͳ���Ѵ��и�Mac��ַ
/*
index: Ӧ���� < MAX_PERIPHERAL_MAC_ADDR,
*/
bool simpleBLE_GetPeripheralMacAddr(uint8 index, uint8 *pAddr);


// �а������£�������Ϊ������ ����Ĭ������Ϊ�ӻ�
// 0 ����peripheral���豸�� 1: ����Ϊ central
bool Check_startup_peripheral_or_central(void);


//����ʱ�жϵ���������3�룬 �ָ���������
//��������Ϊ  p0.7
void CheckKeyForSetAllParaDefault(void); 

// ���п� uart ��ʼ��
void simpleBLE_NPI_init(void);

// ���ý�������
void UpdateRxGain(void);

// ���÷��书��
void UpdateTxPower(void);

// ����led�Ƶ�״̬
void simpleBle_LedSetState(uint8 onoff);

// ����RSSI ��ϵͳ����
void simpleBle_SetRssi(int8 rssi);

// ���ڴ�ӡ����  -----������----
void simpleBle_PrintPassword();

// ��ȡ�豸����
uint8* simpleBle_GetAttDeviceName();

// �����Ƿ��¼�˴ӻ���ַ
bool simpleBle_IFfHavePeripheralMacAddr( void );

// ��ʱ������ʱִ�к����� ��������led��״̬----Ҳ��������һ����ʱ������
void simpleBLE_performPeriodicTask( void );

// ��ȡ��ȨҪ��, 0: ���Ӳ���Ҫ����,  1: ������Ҫ����
bool simpleBle_GetIfNeedPassword();

// ��ȡ��������
uint32 simpleBle_GetPassword();

// �ж��Ƿ��� iBeacon �㲥ģʽ
bool simpleBLE_CheckIfUse_iBeacon();

// �ж��Ƿ�ʹ�ܴ���͸��
bool simpleBLE_CheckIfUse_Uart2Uart();

// �ж���������β�-��ַ�Ƿ�����Ҫȥ���ӵĵ�ַ������ǣ� �����棬 ���򷵻ؼ�
bool simpleBLE_GetToConnectFlag(uint8 *Addr);

// ���� iBeacon �Ĺ㲥���
uint32 simpleBLE_GetiBeaconAdvertisingInterral();

// ���ڻص������� ����Ѹûص�������ʵ�ֵĹ��ܽ���һ��
/*
1, ˼·:  �������յ����ݺ󣬾ͻ����ϵ������»ص���������ʵ�ʲ����з��֣��˻ص�
��������Ƶ���� ����㲻ִ��NPI_ReadTransport�������ж�ȡ�� ��ô����ص������ͻ�
Ƶ���ر�ִ�У����ǣ���ͨ�����ڷ���һ�����ݣ� �㱾�����봦����һ����һ�ε����ݣ����ԣ�
����������������ʱ��Ĵ������� Ҳ�����յ����ݹ�����߳�ʱ���Ͷ�ȡһ�����ݣ� 
Ȼ����ݵ�ǰ��״̬����ִ�У����û�������ϣ��Ͱ��������ݵ���AT����� �������
���ˣ��Ͱ������͵��Զˡ�
*/

//uart �ص�����
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events );


// AT ����� ����
bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length);
// MT ����� ����
bool simpleBLE_MT_CMD_Handle(uint8 *pBuffer, uint16 length);


/*
�ܶ����������ǣ� ���ʵ�ְ�������ӻ��ϵĴ���������ֱ�ӷ��͵��Զ˲�ͨ�������Ĵ���
͸����ȥ�� �������Ǿ���ʵ��������ܣ� ����������Ҫʲô���Ĵ������� �Լ�ʲô��������
����Ҫ���Լ�����֯�ˣ� �����������ÿ100msִ��һ��:
�����԰����ݷ��͵��Զˣ� �Զ�ͨ������͸����ȥ��
�������һ������: ʵ�ְ��ַ������͵��Է�
*/
void simpleBLE_SendMyData_ForTest();


/*
���������������� ������ӻ������������������
ע��ÿ�����������������Ǵӻ�������ͬʱ��������ӻ��ģ� �������ǲ���ͻ��
*/
void simpleBLE_HandleKeys(uint8 keys);

#if defined(USE_DISPLAY_KEY_VALUE)  // ���԰���ר�ã���ʾ5�򰴼�ֵ
void SimpleBLE_DisplayTestKeyValue();
#endif

extern bool g_sleepFlag;    //˯�߱�־
extern uint8 uart_sleep_count; // ˯�߼�����
extern bool g_rssi_flag;       //�Ƿ������


#endif



#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLE_H */
