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
bool g_sleepFlag = TRUE;    //˯�߱�־
uint8 uart_sleep_count = 0; // ˯�߼�����

bool g_rssi_flag = false;       //�Ƿ������
    
extern gaprole_States_t gapProfileState;   // �ӻ�����״̬
// Connection handle
extern uint16 gapConnHandle;

BLE_CENTRAL_CONNECT_CMD g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_NULL;

static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events );
static bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length);
static void simpleBLE_SendMyData_ForTest();
#endif

#if 1
// �ú�����ʱʱ��Ϊ1ms�� ��ʾ������������ ������ ������С  --amomcu.com
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

// �ַ����Ա�
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

// �ַ���ת����
uint32 str2Num(uint8* numStr, uint8 iLength)
{
    uint8 i = 0;
    int32 rtnInt = 0;
 
    /* 
          Ϊ����򵥣���ȷ��������ַ����������ֵ�
          ����£��˴�δ����飬����Ҫ���
          numStr[i] - '0'�Ƿ���[0, 9]���������
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

// �����������ݵ�nv flash
void simpleBLE_WriteAllDataToFlash()
{   // д���в���
    osal_snv_write(0x80, sizeof(SYS_CONFIG), &sys_config); 
}

// ��ȡ�Զ���� nv flash ����  -------δʹ�õ�
void simpleBLE_ReadAllDataToFlash()
{
    int8 ret8 = osal_snv_read(0x80, sizeof(SYS_CONFIG), &sys_config);
}

//flag: PARA_ALL_FACTORY:  ȫ���ָ���������
//flag: PARA_PARI_FACTORY: ��������Ϣ
void simpleBLE_SetAllParaDefault(PARA_SET_FACTORY flag)    
{
    if(flag == PARA_ALL_FACTORY)
    {
        //sys_config.baudrate = HAL_UART_BR_9600;  
        sys_config.baudrate = HAL_UART_BR_115200;  
        sys_config.parity = 0;  
        sys_config.stopbit = 0;  

        sys_config.mode = BLE_MODE_SERIAL;         //����ģʽ 0:͸�� �� 1: ֱ�� , 2: iBeacon

        sprintf((char*)sys_config.name, DEV_NAME_DEFAULT);  //�豸����

        sys_config.role = BLE_ROLE_PERIPHERAL;         //����ģʽ, Ĭ�ϴӻ�
        //sys_config.role = BLE_ROLE_CENTRAL;

        sprintf((char*)sys_config.pass, "000000");      //����
        sys_config.type = 0;                            //��Ȩģʽ
        osal_memset(sys_config.mac_addr, 0, sizeof(sys_config.mac_addr));

        sys_config.ever_connect_peripheral_mac_addr_conut = 0;
        sys_config.ever_connect_peripheral_mac_addr_index = 0;        
        //�����ɹ����ӹ��Ĵӻ���ַ
        osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*MAC_ADDR_CHAR_LEN);

        sys_config.try_connect_time_ms = 0;       // ��������ʱ��---Ŀǰ��Ч 

        sys_config.rssi = 0;                      //  RSSI �ź�ֵ

        sys_config.rxGain = HCI_EXT_RX_GAIN_STD;   //  ��������ǿ��
        sys_config.txPower = 0;                    //  �����ź�ǿ��

        sys_config.ibeacon_adver_time_ms = 500;
        //  ģ�鹤������  0: ���������� 1: �ȴ�AT+CON �� AT+CONNL ����
        sys_config.workMode = 0;          
    }
    else if(flag == PARA_PARI_FACTORY)
    {
        //sprintf((char*)sys_config.pass, "000000");      //����
        osal_memset(sys_config.mac_addr, 0, sizeof(sys_config.mac_addr));
        sys_config.ever_connect_peripheral_mac_addr_conut = 0;
        sys_config.ever_connect_peripheral_mac_addr_index = 0;        
        osal_memset(sys_config.ever_connect_mac_status, 0, MAX_PERIPHERAL_MAC_ADDR*MAC_ADDR_CHAR_LEN);
    }

    GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL ); //�������Ϣ

    simpleBLE_WriteAllDataToFlash();
}

// ��ӡ���д洢��nv flash�����ݣ� ������Դ���
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

    //�����ɹ����ӹ��Ĵӻ���ַ
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

// �����豸��ɫ
//����ģʽ  0: �ӻ�   1: ����
BLE_ROLE GetBleRole()
{
    return sys_config.role;
}

// �ж������Ƿ�������
// 0: δ������
// 1: ��������
bool simpleBLE_IfConnected()
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//����
    {          
        return ( simpleBLEState == BLE_STATE_CONNECTED  );
    }
    else
    {
        return (gapProfileState == GAPROLE_CONNECTED);
    }
}

// ���Ӵӻ���ַ�� ע�⣬ ��Ҫ���ӳɹ��� �����Ӹõ�ַ
void simpleBLE_SetPeripheralMacAddr(uint8 *pAddr)
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//����
    {
       uint8 Addr[MAC_ADDR_CHAR_LEN];
       uint8 index;
       
       //LCD_WRITE_STRING_VALUE( "_conut:", sys_config.ever_connect_peripheral_mac_addr_conut, 10, HAL_LCD_LINE_2 );
       // Ҫ���Ӵӻ���ַ�� ��Ҫ�ȼ�����ǵĵ�ַ�б����Ƿ��Ѿ����иĵ�ַ�� ����Ѿ����ˣ� ��ô����������
       for(index = 0; index < sys_config.ever_connect_peripheral_mac_addr_conut; index++)        
       {
          if(simpleBLE_GetPeripheralMacAddr(index, Addr))
          {
             if(osal_memcmp(Addr, pAddr, MAC_ADDR_CHAR_LEN))//��ַһ��ʱֱ�ӷ���
             {
                //LCD_WRITE_STRING_VALUE( "_index: return", index, 10, HAL_LCD_LINE_2 );
                //����һ�γɹ����ӹ��Ĵӻ���ַindex�� �������AT+CONNL ���ָ��
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
       
       //ֻ�������Ĵӻ���ַ�� ������, �������ȵ�һ����ַ��¼���б���ֻ���������     MAX_PERIPHERAL_MAC_ADDR    ����ַ
       osal_memcpy(sys_config.ever_connect_mac_status[sys_config.ever_connect_peripheral_mac_addr_index], pAddr, MAC_ADDR_CHAR_LEN);

       //����һ�γɹ����ӹ��Ĵӻ���ַindex�� �������AT+CONNL ���ָ��
       sys_config.last_connect_peripheral_mac_addr_index = sys_config.ever_connect_peripheral_mac_addr_index;

       sys_config.ever_connect_peripheral_mac_addr_index++;
       // ע����������ļ����÷�
       sys_config.ever_connect_peripheral_mac_addr_index %= MAX_PERIPHERAL_MAC_ADDR; 

       // ֻ��¼  MAX_PERIPHERAL_MAC_ADDR �����µĵ�ַ
       if(sys_config.ever_connect_peripheral_mac_addr_conut < MAX_PERIPHERAL_MAC_ADDR) 
       {
         sys_config.ever_connect_peripheral_mac_addr_conut++; 
       }

       //LCD_WRITE_STRING_VALUE( "_conut2:", sys_config.ever_connect_peripheral_mac_addr_conut, 10, HAL_LCD_LINE_2 );

       // �����ַ �� �Ա��������Ӻ����Ҳʹ�� 
       simpleBLE_WriteAllDataToFlash();
    }
}

// ��ȡ�ӻ���ַ, index < MAX_PERIPHERAL_MAC_ADDR
// �����ж��Ƿ�ϵͳ���Ѵ��и�Mac��ַ
/*
index: Ӧ���� < MAX_PERIPHERAL_MAC_ADDR,
*/
bool simpleBLE_GetPeripheralMacAddr(uint8 index, uint8 *pAddr)
{
    if(GetBleRole() == BLE_ROLE_CENTRAL)//����
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


// �а������£�������Ϊ������ ����Ĭ������Ϊ�ӻ�
// 0 ����peripheral���豸�� 1: ����Ϊ central
bool Check_startup_peripheral_or_central(void) 
{
    P0SEL &= ~0x02;     //����P0.1Ϊ��ͨIO��  
    P0DIR &= ~0x02;     //��������P0.1���ϣ���P0.1Ϊ����ģʽ 
    P0INP &= ~0x02;     //��P0.1��������

    if(0 == P0_1)// �а�������
    {
        // 10ms ȥ���� 
        simpleBLE_Delay_1ms(10);
        if(0 == P0_1)// �а�������
        {
            return true;
        }
    }

    return false;
}


//����ʱ�жϵ���������3�룬 �ָ���������
//��������Ϊ  p0.7
void CheckKeyForSetAllParaDefault(void) 
{
#if 0  
    uint8 i;
    uint32 old_time  = 30; 

    P0SEL &= ~0x02;     //����P0.1Ϊ��ͨIO��  
    P0DIR &= ~0x02;     //��������P0.1���ϣ���P0.1Ϊ����ģʽ 
    P0INP &= ~0x02;     //��P0.1��������


    //�ж�3s �Ȱ�����û���ɿ�
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

    //�ж�3s �Ȱ�����û���ɿ��� ��ô���лָ��������ã�����������������ٻָ���������Ȼ������
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
        // ������ ʵ���������ÿ��Ź���
        HAL_SYSTEM_RESET();     
    }
#endif

#if 0
    if(P0_1 == 0)// �а�������
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

// ���п� uart ��ʼ��
void simpleBLE_NPI_init(void)
{
#if 1    
    NPI_InitTransportEx(simpleBLE_NpiSerialCallback, sys_config.baudrate, 
    sys_config.parity, sys_config.stopbit );
#else
    NPI_InitTransport(simpleBLE_NpiSerialCallback);
#endif

    // ������ӡ�������Ǵӻ�
    if(GetBleRole() == BLE_ROLE_CENTRAL)
    {
        NPI_WriteTransport("Hello World Central\r\n",21);
    }
    else
    {
        NPI_WriteTransport("Hello World Peripheral\r\n",24);
    }
}

// ���ý�������
void UpdateRxGain(void)
{
    // HCI_EXT_SetRxGainCmd()���������÷��书�ʵ�. 
    // rxGain - HCI_EXT_RX_GAIN_STD, HCI_EXT_RX_GAIN_HIGH
    HCI_EXT_SetRxGainCmd( HCI_EXT_RX_GAIN_STD );
}

// ���÷��书��
void UpdateTxPower(void)
{
        /*
#define LL_EXT_TX_POWER_MINUS_23_DBM                   0
#define LL_EXT_TX_POWER_MINUS_6_DBM                    1
#define LL_EXT_TX_POWER_0_DBM                          2
#define LL_EXT_TX_POWER_4_DBM                          3
        */
    // HCI_EXT_SetTxPowerCmd()���������÷��书�ʵ�. ��-23dbm, -6dbm, 0 dbm, +4dbm�ĸ�����. 
//    uint8 txPower;
//#if defined(CC2541)
    HCI_EXT_SetTxPowerCmd(3 - sys_config.txPower);
//#endif
}

// ����led�Ƶ�״̬
void simpleBle_LedSetState(uint8 onoff)
{
  HalLedSet( HAL_LED_1, onoff);  //led����

  P0DIR |= 0x60;  // P0.6����Ϊ���
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

// �󻬶�ƽ��ֵ
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

         // ȥ�������Сֵ, ����ƽ��
         
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

// ����RSSI ��ϵͳ����
void simpleBle_SetRssi(int8 rssi)
{
    sys_config.rssi = rssi;

    if(simpleBLE_IfConnected())
    {
        char str[32];    

        float nfDist = GUA_CalcDistByRSSI(rssi);         //ͨ���㷨���r����λΪm  
        int nDist = (int)(nfDist * 100);                    //��r����ֵ�Ŵ�100������λΪcm  
        sprintf(str, "Rssi=%2d,%4dCM\r\n", (uint8) (-rssi), dist_filer(nDist));

        if(g_rssi_flag)
        {
            NPI_WriteTransport((uint8*)str, strlen(str));

            //���͵��Զˡ� �����ֻ�
            qq_write((uint8*)str, strlen(str));
            // �����¼���Ȼ�����¼�����������ʱ����ʱ������ݲ����͵�����
            osal_set_event( simpleBLETaskId, SBP_UART_EVT );     
        }

        LCD_WRITE_STRING(str, HAL_LCD_LINE_5 );
    }  
}

// ���ڴ�ӡ����  -----������----
void simpleBle_PrintPassword()
{
    char strTemp[24] = {0};
    
    sprintf(strTemp, "Password:%s\r\n", sys_config.pass);
    NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));
}

// ��ȡ�豸����
uint8* simpleBle_GetAttDeviceName()
{
    return sys_config.name;
}

// �����Ƿ��¼�˴ӻ���ַ
bool simpleBle_IFfHavePeripheralMacAddr( void )
{
    if(sys_config.ever_connect_peripheral_mac_addr_conut > 0)
        return TRUE;
    else
        return TRUE;
}

// ��ʱ������ʱִ�к����� ��������led��״̬----Ҳ��������һ����ʱ������
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
//        simpleBle_LedSetState(HAL_LED_MODE_TOGGLE);           //ȡ��		
//        return;
//    }
    
    /*
    ����ǰ��
        ����δ��¼�ӻ���ַʱ��ÿ����100ms��
        ������¼�ӻ���ַʱ��ÿ����900ms��
        �ӻ�ÿ2����1�롣
    ���ߺ�
        ������ӻ���Ϊ��LEDÿ5����100���롣
    */
    if(!simpleBLE_IfConnected())
    {    
        if(GetBleRole() == BLE_ROLE_CENTRAL)//����
        {     
            if(simpleBle_IFfHavePeripheralMacAddr() == FALSE)//δ��¼��ַ
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
        else//�ӻ�
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

#if defined(USE_DISPLAY_KEY_VALUE)  // ���԰���ר�ã���ʾ5�򰴼�ֵ
        SimpleBLE_DisplayTestKeyValue();       
#endif
    }
    else// ���Ӻ� ������ӻ���Ϊ��LEDÿ5����100���롣(�����ʡ�磬 ���Բ����)
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

        // ��1min����һ����ʪ��ֵ
        simpleBLE_SendMyData_ForTest();
    }
}

// ��ȡ��ȨҪ��, 0: ���Ӳ���Ҫ����,  1: ������Ҫ����
bool simpleBle_GetIfNeedPassword()
{
    return sys_config.type;
}

// ��ȡ��������
uint32 simpleBle_GetPassword()
{
    uint32 passcode;   
    
    passcode = str2Num(sys_config.pass, 6);
    passcode %= 1000000;

    return passcode;
}

// �ж��Ƿ��� iBeacon �㲥ģʽ
bool simpleBLE_CheckIfUse_iBeacon()
{
    return (sys_config.mode == BLE_MODE_iBeacon);
}

// �ж��Ƿ�ʹ�ܴ���͸��
bool simpleBLE_CheckIfUse_Uart2Uart()
{
    return (sys_config.mode == BLE_MODE_SERIAL);
}

// �ж���������β�-��ַ�Ƿ�����Ҫȥ���ӵĵ�ַ������ǣ� �����棬 ���򷵻ؼ�
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

// ���� iBeacon �Ĺ㲥���
uint32 simpleBLE_GetiBeaconAdvertisingInterral()
{
    return sys_config.ibeacon_adver_time_ms;    
}

#if 1
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
static void simpleBLE_NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port;

    static uint32 old_time;     //��ʱ��
    static uint32 old_time_data_len = 0;     //��ʱ���ǵ����ݳ���    
    uint32 new_time;            //��ʱ��
    bool ret;
    uint8 readMaxBytes = SIMPLEPROFILE_CHAR6_LEN;

    if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))   //����������
    {
        (void)port;
        /*uint8*/uint16 numBytes = 0;

        uart_sleep_count = 0;
        
#if 1
        if(TRUE == g_sleepFlag)//����˯����...�����ݶ�ȡ��������������
        {
            numBytes = NPI_RxBufLen();           //�������ڻ������ж����ֽ�
            if(numBytes > 0)
            {
                // ע�Ȿ�������������ã� ����������ֻ��ȡ�������ݶ��ǿ��Ե�
                numBytes = (numBytes > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : numBytes;                    
                uint8 *buffer = osal_mem_alloc(numBytes);
                NPI_ReadTransport(buffer,numBytes);    //�ͷŴ�������    
                osal_mem_free(buffer);
            }
            
            return;
        }
        else
        {
            numBytes = NPI_RxBufLen();           //�������ڻ������ж����ֽ�
        }
#else
        numBytes = NPI_RxBufLen();           //�������ڻ������ж����ֽ�
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
            old_time = osal_GetSystemClock(); //��������ʱ�� ��¼һ��
            old_time_data_len = numBytes;
        }
        else
        {
            // ע��: δ������ʱ�� ��ЩAT ����Ƚϳ��� ������Ҫ���ٽϴ�Ļ�����
            //       �������Ժ� �յ�ÿһ�ܷ��͵����ݲ����� SIMPLEPROFILE_CHAR6_LEN ���ֽ���������
            //       ��ˣ�����Ҫ����һ��
            if(!simpleBLE_IfConnected())
            {
               readMaxBytes = 22 ;    //���ֵ�� һ�����ó� AT ����������ֽ������ɣ� (����"\r\n" ����)
            }
            else
            {
               readMaxBytes = SIMPLEPROFILE_CHAR6_LEN;
            }

            
            new_time = osal_GetSystemClock(); //��ǰʱ��
            
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
                    NPI_ReadTransport(buffer,sendBytes);    //�ͷŴ�������    
                    
                    if(sendBytes > 2 
                    && buffer[sendBytes-2] == '\r' 
                    && buffer[sendBytes-1] == '\n')
                    {//��⵽ \r\n �������ַ����� ������ AT ����
                        ret = simpleBLE_AT_CMD_Handle(buffer, sendBytes);
                    }
                    else
                    {
                        ret = FALSE;
                    }
                    
                    if(ret == FALSE)
                    {
                        char strTemp[12];
                        //�������� ֱ�ӷ��� "ERROR\r\n"�� �����κβ�������
                        sprintf(strTemp, "ERROR\r\n");
                        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                    }
                }
                else
                {
                    NPI_ReadTransport(buffer,sendBytes);    //�ͷŴ�������   
                    qq_write(buffer, sendBytes);

                    // �����¼���Ȼ�����¼�����������ʱ����ʱ������ݲ����͵�����
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
// AT ����� ����
bool simpleBLE_AT_CMD_Handle(uint8 *pBuffer, uint16 length)
{
    bool ret = TRUE;
    char strTemp[64];
    uint8 i;
    uint8 temp8;  
    bool restart = FALSE;
 
    //NPI_WriteTransport((uint8*)pBuffer, length); 
    // 1������
    if((length == 4) && str_cmp(pBuffer, "AT\r\n", 4))//AT    
    {
        sprintf(strTemp, "OK\r\n");
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // ��ӡ���õĲ���  for test only
    else if((length == 8) && str_cmp(pBuffer, "AT+ALL\r\n", 8))//AT    
    {
        PrintAllPara();
    }
    // 2����ѯ�����ò�����
    else if((length == 10) && str_cmp(pBuffer, "AT+BAUD", 7))
    {
        /*
        ���ͣ�AT+BAUD2 
        ���أ�OK+Set:2 
        0---------9600 
        1---------19200 
        2---------38400 
        3---------57600 
        4---------115200
        */
        switch(pBuffer[7])
        {
        case '?':  //��ѯ��ǰ������
            sprintf(strTemp, "OK+Get:%d\r\n", sys_config.baudrate);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':  //��ѯ�����µĲ�����
            sys_config.baudrate = pBuffer[7] - '0';
            simpleBLE_WriteAllDataToFlash();
            sprintf(strTemp, "OK+Set:%d\r\n", sys_config.baudrate);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            restart = TRUE;  //ֱ����������
            break;
        default:
            ret = FALSE;            
            break;
        }        
    }
    // 3�����ô���У��
    else if((length == 10) && str_cmp(pBuffer, "AT+PARI", 7))
    {   
        // �����ù���
        ret = FALSE;            
    }    
    // 4������ֹͣλ
    else if((length == 10) && str_cmp(pBuffer, "AT+STOP", 7))
    {
        // �����ù���
        ret = FALSE;            
    }       
    // 5. ����ģ�鹤��ģʽ
    else if((length == 10) && str_cmp(pBuffer, "AT+MODE", 7))
    {
        /*
        Para: 0 ~ 1
        0: ��������͸��ģʽ
        1: �رմ���͸��ģʽ
        2: iBeacon �㲥ģʽ
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

            restart = TRUE;  //ֱ����������
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }          
    // 6����ѯ�������豸����
    else if((length >=10 && length <= 20) && str_cmp(pBuffer, "AT+NAME", 7))
    {
        /*
        Para1���豸����
        � 11 λ���ֻ���ĸ��
        ���л��ߺ��»��ߣ�����
        ���������ַ���
        Default��DEV_NAME_DEFAULT(���궨��)
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

            restart = TRUE;  //ֱ����������
            break;
        }        
    }    
    //7. �ָ�Ĭ������(Renew)
    else if((length == 10) && str_cmp(pBuffer, "AT+RENEW", 8))
    {
        sprintf(strTemp, "OK+RENEW\r\n");
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        
        simpleBLE_SetAllParaDefault(PARA_ALL_FACTORY);

        restart = TRUE;  //ֱ����������
    }
    //8. ģ�鸴λ������(Reset)
    else if((length == 10) && str_cmp(pBuffer, "AT+RESET", 8))
    {
        restart = TRUE;  //ֱ����������
    }
    // 9����ѯ����������ģʽ
    else if((length == 10) && str_cmp(pBuffer, "AT+ROLE", 7))
    {
        /*
        Para1: 0 ~ 1 
        1: ���豸
        0: ���豸
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

            restart = TRUE;  //ֱ����������
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }    
    // 10�� ��ѯ�������������
    else if(((length == 10) && str_cmp(pBuffer, "AT+PASS?", 8))
        || ((length == 15) && str_cmp(pBuffer, "AT+PASS", 7)))
    {
        /*
        Para1: 000000~999999 
        Default��000000
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
    // 11�� ����ģ���Ȩ��������
    else if((length == 10) && str_cmp(pBuffer, "AT+TYPE", 7))
    {
        /*
        Para: 0 ~ 1 
        0: ���Ӳ���Ҫ����
        1: ������Ҫ����
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

            restart = TRUE;  //ֱ����������
            break;
        default:    
            ret = FALSE;            
            break;
        }        
    }       
    // 12�� ��ѯ���� MAC ��ַ
    else if((length >= 10) && str_cmp(pBuffer, "AT+ADDR?", 8))
    {        
        sprintf(strTemp, "OK+LADD:%s\r\n", sys_config.mac_addr);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }    
    // 13�� �������һ�����ӳɹ��Ĵ��豸
    else if((length == 10) && str_cmp(pBuffer, "AT+CONNL", 8))
    {
        /*
        Para: L, N, E,F
        L:�����С�N:�յ�ַ
        E:���Ӵ���F:����ʧ��
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
                
                  // ��ʼɨ��
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
    // 14������ָ��������ַ�����豸����豸
    else if((length == 20) && str_cmp(pBuffer, "AT+CON", 6))
    {
        /*
        Para1: MAC��ַ��
        ��: 0017EA0923AE
        Para2: A, E, F
        A: ������
        E: ���Ӵ���
        F: ����ʧ��
        */
        uint8 para[3] = {'A','E','F'};
        uint8 id = 0;
        
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {
            g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_CON;
            osal_memcpy(sys_config.connect_mac_addr, pBuffer+6, MAC_ADDR_CHAR_LEN);

            sprintf(strTemp, "AT+CONN%c\r\n", para[id]);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            // ��ʼɨ��
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
    // 15�� ������豸�����Ϣ
    else if((length == 10) && str_cmp(pBuffer, "AT+CLEAR", 8))
    {
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {        
            simpleBLE_SetAllParaDefault(PARA_PARI_FACTORY);
            //PrintAllPara();

            // ��������� ϵͳ API ������ ɾ�����������Ϣ
            GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
            
            sprintf(strTemp, "OK+CLEAR\r\n");
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
        }
        else
        {
            ret = FALSE;
        }
    }
    // 16����ѯ�ɹ����ӹ��Ĵӻ���ַ 
    // ��ע����ָ��ֻ�������豸ʱ����Ч�����豸ʱ�����ܴ�ָ��
    else if((length == 10) && str_cmp(pBuffer, "AT+RADD?", 8) 
        && (BLE_ROLE_CENTRAL == GetBleRole()))//��������Ч
    {
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {        
            if(sys_config.ever_connect_peripheral_mac_addr_conut == 0)//�޵�ַ
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
    // 17�� ��ѯ����汾
    else if((length == 10) && str_cmp(pBuffer, "AT+VERS?", 8))
    {
        sprintf(strTemp, "%s\r\n", VERSION);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
    }
    // 18�� ������ģʽ�³�������ʱ��
    else if((length == 10) && str_cmp(pBuffer, "AT+TCON", 7))
    {
        /*
        ָ��	                Ӧ��	            ����
        ��ѯ��AT+TCON?	        OK+TCON:[para] 	
        ���ã�AT+TCON[para]	    OK+Set:[para] 	    Para: 000000��009999 
                                                    000000 ����������ӣ���
                                                    ������Եĺ�����
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
    // 19�� ��ȡ RSSI �ź�ֵ
    else if((length == 10) && str_cmp(pBuffer, "AT+RSSI?", 10))
    {
        sprintf(strTemp, "OK+RSSI:%d\r\n", sys_config.rssi);
        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp));        
    }
    // 20��  �ı�ģ�鷢���ź�ǿ��
    else if((length == 10) && str_cmp(pBuffer, "AT+TXPW", 7))
    {
        /*
        ָ��	        Ӧ��	            ����
        ��ѯ��          AT+TXPW?	        OK+ TXPW:[para]	
        ���ã�          AT+TXPW[para]	    OK+Set:[para]	Para: 0 ~ 3
                                            0: 4dbm��1: 0dbm
                                            2: -6dbm��3: -23dbm
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
            // HCI_EXT_SetTxPowerCmd()���������÷��书�ʵ�. ��-23dbm, -6dbm, 0 dbm, +4dbm(��CC2540)�ĸ�����. 
            HCI_EXT_SetTxPowerCmd(3 - sys_config.txPower);
#endif      

            restart = TRUE;  //ֱ����������
         }
    }        
    // 21��  �ı�ģ����Ϊibeacon��վ�㲥ʱ����
    else if((length == 10 || length == 15) && str_cmp(pBuffer, "AT+TIBE", 7))
    {
        /*
        ָ��	        Ӧ��	        ����
        ��ѯ��          AT+TIBE?	    OK+ TIBE:[para]	
        ���ã�          AT+TIBE[para]	OK+Set:[para]	Para: 000000��009999 
                                        000000 ��������㲥����
                                        ������Եĺ�����
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

                    restart = TRUE;  //ֱ����������
                }
            }
        }
        else
        {
            ret = FALSE;
        }        
    }
    // 22��  ����ģ�鹤������ 
    else if((length == 10) && str_cmp(pBuffer, "AT+IMME", 7))
    {
        /*
        ָ��	        Ӧ��	        ����
        ��ѯ��          AT+IMME?	    OK+Get:[para]	Para: 0~1
        ���ã�          AT+IMME[para]	OK+Set:[para]	Para: 0~1
                                        000000 ��������㲥����
                                        0: ���������� 
                                        1: �ȴ�AT+CON �� AT+CONNL ����
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

            restart = TRUE;  //ֱ����������
        }
    }    
    // 23�� ���ÿ�ʼ��ģʽ�µĴӻ�ɨ��
    else if((length == 10) && str_cmp(pBuffer, "AT+DISC?\r\n", 10))
    {
        /*
        AT+DISC? OK+DISCS ��
        ��ָ��Ϊ��ģʽ���ֶ�����ģʽָ�ʹ��ǰ������ AT+ROLE1��AT+IMME1
        ������
        ���� AT+DISC?ָ���ģ�������ڴ���״̬������ OK+DISCS ��ʼ������
        ����ѵ� BLE �豸�᷵�� OK+DISC:123456789012,���������ַ��������
        ��������ַ����෵�� 6 �����±�� 0 ��ʼ��������ɺ󷵻� OK+DISCE��
        ��ֻ�ѵ������豸�����ӣ�
        ����: AT+DISC?
        ����: OK+DISCS
        ����: OK+DISC:123456789012
        ����: OK+DISCE
        ���Ҫ�����ѵ����豸���±�� 0 ��ʼ��
        ����: AT+CONN0 �����ѵ��ĵ�һ���豸
        ����: AT+CONN1 �����ѵ��ĵڶ����豸
        ��������
        */
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {
            g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_DISC;

            sprintf(strTemp, "OK+DISCS\r\n");
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            // ��ʼɨ��
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
    // 24�� �����Ѿ����ֵĴӻ��� ��������±��
    else if((length == 10) && str_cmp(pBuffer, "AT+CONN", 7))
    {
        /* 
        ����: AT+CONN0 �����ѵ��ĵ�һ���豸
        ����: AT+CONN1 �����ѵ��ĵڶ����豸
        ��������
        */
        if((GetBleRole() == BLE_ROLE_CENTRAL))
        {
            //sprintf(strTemp, "OK+DISCS\r\n");
            //NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 

            g_Central_connect_cmd  = BLE_CENTRAL_CONNECT_CMD_CONN;

            temp8 = pBuffer[7] - '0';
            // ��ʼɨ��
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
    // 25�� AT+ADC ��ȡһ·��ѹֵ����
//    else if((length == 10) && str_cmp(pBuffer, "AT+SLEEP", 8))
//    {
//        g_sleepFlag = TRUE;
//        osal_pwrmgr_device( PWRMGR_BATTERY);   //  �Զ�˯��
//        //osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  ��˯�ߣ����ĺܸߵ�        
//        // ��ʽ��
//        sprintf(strTemp, "+SLEEP\r\nOK\r\n");
//        NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
//        {
//            //����p02�Ĺ��ܣ�1Ϊuart�ţ� 0Ϊ�����жϽ�
//            extern void HalKey_SET_P02_UART_RX(bool flag);

//            // ������ʱ 10ms                    
//            simpleBLE_Delay_1ms(10);
//            
//            HalKey_SET_P02_UART_RX(false);
//        }
//    }
    // 25�� AT+ADC ��ȡһ·��ѹֵ����
    else if((length == 9) && str_cmp(pBuffer, "AT+ADC", 6))
    {
        uint8 ch = pBuffer[6];  // AT+ADC��β�ţ�����AT+ADC4�е�4
        uint16 adc;         // adc ����    
        float volt;         // ��ѹֵ
        uint8 channel;      // ͨ��
        
        switch(ch)
        {
        case '4'://����P0.4�˿ڵ�ѹ adc   
            channel = HAL_ADC_CHN_AIN4; goto NEXT_ADC;
        case '5'://����P0.5�˿ڵ�ѹ adc
            channel = HAL_ADC_CHN_AIN5; goto NEXT_ADC;
        case '6'://����P0.6�˿ڵ�ѹ adc
            channel = HAL_ADC_CHN_AIN6; goto NEXT_ADC;
        case '7'://����P0.7�˿ڵ�ѹ adc
            channel = HAL_ADC_CHN_AIN7; goto NEXT_ADC;
NEXT_ADC:
            HalAdcSetReference( HAL_ADC_REF_AVDD );
            adc = HalAdcRead( channel, HAL_ADC_RESOLUTION_14 ); //HAL_ADC_RESOLUTION_14  ����ЧλΪ13λ���뿴�ú����ڲ�����
            volt = adc*3.3/8192;
            // ��ʽ��
            sprintf(strTemp, "OK+Get:ADC%d=%d,%.2fv\r\n", ch-'0',adc, volt);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        case '8'://���������ѹ
        {
            // �ο��Ա�Э��ջ�е� Battservice.c �ļ�
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
                // �ٷֱ�
                percent = (uint8) ((((adc - battMinLevel) * 25) + (range - 1)) / range);
            }
            // ��ʽ��
            sprintf(strTemp, "OK+Get:ADC%d=%d,%d%%\r\n", ch-'0',adc, percent);
            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
            break;
        }
        case '9'://����оƬ�ڲ��¶�
        {
            /*
            �ο���ַ
            https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/303585
            */
#define TEMP_CONST 0.61065 // (1250 / 2047) 
#define TEMP_OFFSET_DATASHEET 750 
#define TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS 29.75 
#define TEMP_OFFSET (TEMP_OFFSET_DATASHEET + TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS) // 779.75 
#define TEMP_COEFF 4.5  

            float AvgTemp;   
            float AvgTemp_sum = 0;               

            TR0 = 0x01;                     //����Ϊ1�������¶ȴ�������SOC_ADC
            ATEST = 0x01;                   //ʹ���¶ȴ���            

            // ��64��ƽ��ֵ���Ա��ȶ�
            for(i = 0; i<64; i++)
            {
                HalAdcSetReference( HAL_ADC_REF_125V );
                adc = HalAdcRead( HAL_ADC_CHN_TEMP, HAL_ADC_RESOLUTION_12 );
                AvgTemp = adc * TEMP_CONST; 
                AvgTemp =  ((AvgTemp - TEMP_OFFSET) / TEMP_COEFF); 
                AvgTemp_sum += AvgTemp;
            }
            // �õ�ʵ���¶�
            AvgTemp = AvgTemp_sum/64; 
            
            TR0 &= ~0x01;
            ATEST &= ~0x01;
            
            // ��ʽ��
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

    if(restart)//����ñ�־�����ã���΢��ʱ������
    {
        simpleBLE_Delay_1ms(200);      //���ò������ʵ���ʱ�� �Ա���һ�η��͵������������ͳ�ȥ
        HAL_SYSTEM_RESET(); 
    }
    
    return ret;
}
#endif

#if 1
/*
�ܶ����������ǣ� ���ʵ�ְ�������ӻ��ϵĴ���������ֱ�ӷ��͵��Զ˲�ͨ�������Ĵ���
͸����ȥ�� �������Ǿ���ʵ��������ܣ� ����������Ҫʲô���Ĵ������� �Լ�ʲô��������
����Ҫ���Լ�����֯�ˣ� �����������ÿ100msִ��һ��:
�����԰����ݷ��͵��Զˣ� �Զ�ͨ������͸����ȥ��
�������һ������: ʵ�ְ��ַ������͵��Է�
*/
void simpleBLE_SendMyData_ForTest()
{
    uint8 buffer[2] = {0};
    
    static uint16 count_100ms = 0;
    count_100ms++;
    
    if(count_100ms >= 600){//60s 
        
    DHT11();           //��ȡ��ʪ��

    //����ʪ�ȵ�ת�����ַ���
//    temp[0]=wendu_shi+0x30;
//    temp[1]=wendu_ge+0x30;
//    humidity[0]=shidu_shi+0x30;
//    humidity[1]=shidu_ge+0x30;
        
    //��õ���ʪ��ͨ�����������������ʾ
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

    // �������һЩ����
    
    count_100ms++;
    if(count_100ms == 1)//������ÿ100ms��ִ��һ�Σ� ����10�ξ���1s
    {
        char strTemp[24] = {0};

        if((GetBleRole() == BLE_ROLE_CENTRAL) && simpleBLEChar6DoWrite && simpleBLECentralCanSend)               
        {
            sprintf(strTemp, "[%8ldms]Amo1\r\n", osal_GetSystemClock());
            //�����������֯�� strTemp�� ��ok��, ע�ⲻҪ���� SIMPLEPROFILE_CHAR6_LEN �Ĵ�С
            //����㷢�͵�����Ҫ���� SIMPLEPROFILE_CHAR6_LEN�� ��ô��õİ취�� ��������һ����ʱ����Ȼ��ÿ��ʱ������ �ͷ���һ������
            // ��ʱ���������� ��ο�       osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
            numBytes = (osal_strlen(strTemp) > SIMPLEPROFILE_CHAR6_LEN) ? SIMPLEPROFILE_CHAR6_LEN : osal_strlen(strTemp);            
            simpleBLE_UartDataMain((uint8*)strTemp, numBytes);
        }
        else if((GetBleRole() == BLE_ROLE_PERIPHERAL) && simpleBLEChar6DoWrite2)                    
        {
            sprintf(strTemp, "[%8ldms]Amo2\r\n", osal_GetSystemClock());
            //�����������֯�� strTemp�� ��ok��, ע�ⲻҪ���� SIMPLEPROFILE_CHAR6_LEN �Ĵ�С
            //����㷢�͵�����Ҫ���� SIMPLEPROFILE_CHAR6_LEN�� ��ô��õİ취�� ��������һ����ʱ����Ȼ��ÿ��ʱ������ �ͷ���һ������
            // ��ʱ���������� ��ο�       osal_start_timerEx( simpleBLETaskId, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
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
    | HAL_KEY_LEFT | HAL_KEY_DOWN );     // ���ڰ������ԣ��������Է������
/*
���������������� ������ӻ������������������
ע��ÿ�����������������Ǵӻ�������ͬʱ��������ӻ��ģ� �������ǲ���ͻ��
*/
void simpleBLE_HandleKeys(uint8 keys)
{
    static char key_info[12] = {0};         // ���水����Ϣ
    static char key_info2[20] = {0};        // ����ר�ã�������ʾ����ʽ���ȽϺÿ�
    bool key_pressed = true;                // �������±��
    char str[24];                           // ������

    // ��¼���ְ�����Ϣ
    if ( keys & HAL_KEY_SW_6 )
    {
        keyTestflag &= (~HAL_KEY_SW_6);     // �����ð�������
        sprintf(key_info, "KEY_S1");
        sprintf(key_info2, "     KEY_S1");

        g_sleepFlag = !g_sleepFlag;   
        if(g_sleepFlag)
        {
            osal_pwrmgr_device( PWRMGR_BATTERY);   //  �Զ�˯��
        }
        else
        {
            osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  ��˯�ߣ����ĺܸߵ�        
        }

        /* ˯�߱�־
        ��ӡ Sleep 0 ��ʾû��˯�ߣ���ʱ���Ľϸߣ����ڵ�rx�߿����յ�����
        ��ӡ Sleep 1 ��ʾ����˯�ߣ���ʱ���Ľϵͣ����ڵ�rx�߲������յ�����
        */
        sprintf(str, "Sleep %d ", g_sleepFlag);
        HalLcdWriteString(str, HAL_LCD_LINE_6);

        NPI_WriteTransport_ln((uint8*)str, strlen(str)); // ͨ�����ڷ��ͳ���
        simpleBLE_Delay_1ms(1);
    }
#if defined ( AMOMCU_UART_RX_MODE)
    else if ( keys & HAL_KEY_SW_7 )
    {        
        sprintf(key_info, "KEY_S7");

        {
            //����p02�Ĺ��ܣ�1Ϊuart�ţ� 0Ϊ�����жϽ�
            HalKey_Set_P02_for_UartRX_or_GPIO(true);
            //osal_pwrmgr_device( PWRMGR_BATTERY);   //  �Զ�˯��
            osal_pwrmgr_device( PWRMGR_ALWAYS_ON);   //  ��˯�ߣ����ĺܸߵ�
            osal_start_timerEx( simpleBLETaskId, SBP_WAKE_EVT, 10);// 10ms�����ݲ���Ч�� ����м��10���������ڽ��ջ��Ѵ��ڵ����ݣ� ��������Ϊ��Ч
        }        
    }
#endif
    else if ( keys & HAL_KEY_UP )
    {
        keyTestflag &= (~HAL_KEY_UP);     // �����ð�������

        sprintf(key_info, "KEY_UP");
        sprintf(key_info2, "  KEY_UP");


        if(GetBleRole() == BLE_ROLE_CENTRAL)//����
        {
            if(simpleBLEState != BLE_STATE_CONNECTED)
            {   // �������豸
                simpleBLE_AT_CMD_Handle("AT+DISC?\r\n", 10);
            }
        }
    }
    else if ( keys & HAL_KEY_DOWN )
    {
        keyTestflag &= (~HAL_KEY_DOWN);     // �����ð�������

        sprintf(key_info, "KEY_DOWN");
        sprintf(key_info2, "      KEY_DOWN");

        // �򿪻��߹ر�rssi��๦��
        g_rssi_flag = !g_rssi_flag;
    }  
    else if ( keys & HAL_KEY_LEFT )
    {
        keyTestflag &= (~HAL_KEY_LEFT);     // �����ð�������

        sprintf(key_info, "KEY_LEFT");
        sprintf(key_info2, "KEY_LEFT");

        if(GetBleRole() == BLE_ROLE_CENTRAL)//����
        {
            if(simpleBLEState != BLE_STATE_CONNECTED)
            {   // ��ʾ��һ�����豸 
                // ��ʾ��һ�����豸�ĵ�ַ  nextFalg=true����ʾ��һ����ַ��������ʾ��ǰ��ַ
                extern void simpleBLECentraDisplaNextPeriAddr(bool nextFalg);
                simpleBLECentraDisplaNextPeriAddr(TRUE);   
            }
        }
    }  
    else if ( keys & HAL_KEY_RIGHT )
    {
        keyTestflag &= (~HAL_KEY_RIGHT);     // �����ð�������

        sprintf(key_info, "KEY_RIGHT");
        sprintf(key_info2, "        KEY_RIGHT");
    }  
    else if ( keys & HAL_KEY_CENTER )
    {
        char buffer[12];

        keyTestflag &= (~HAL_KEY_CENTER);     // �����ð�������

        sprintf(key_info, "KEY_CENTER");
        sprintf(key_info2, "    KEY_CENTER");

        if(GetBleRole() == BLE_ROLE_CENTRAL)//����
        {
            if(simpleBLEState != BLE_STATE_CONNECTED)
            {   // ���ӵ�ǰ��������ʾ��mac��ַ�Ĵ��豸�� ������������
                extern uint8 simpleBLECentraGetAddrId();
                uint8 id = simpleBLECentraGetAddrId();
                sprintf(buffer, "AT+CONN%d\r\n", id);
                simpleBLE_AT_CMD_Handle((uint8*)buffer, strlen(buffer));
            }
            else if(simpleBLE_IfConnected()) 
            {
                // ���Դ��룬����һ��ָ� ������Ϩ����豸��LED2
                static uint8 led2OnOff = 0;
                led2OnOff = led2OnOff ? 0 : 1;
                
                sprintf(buffer, "AT+LED2=%d\r\n", led2OnOff);
                qq_write((uint8*)buffer, strlen(buffer));

                // �����¼���Ȼ�����¼�����������ʱ����ʱ������ݲ����͵�����
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );
            }
        } 
        else // ���豸�����淢�͵��ָ����������� LED3
        {
            if(simpleBLE_IfConnected()) 
            {
                // ���Դ��룬����һ��ָ� ������Ϩ�����豸��LED3
                static uint8 led2OnOff = 0;
                led2OnOff = led2OnOff ? 0 : 1;
                
                sprintf(buffer, "AT+LED3=%d\r\n", led2OnOff);
                qq_write((uint8*)buffer, strlen(buffer));

                // �����¼���Ȼ�����¼�����������ʱ����ʱ������ݲ����͵�����
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );
            }
        }
    }
    else
    {
        key_pressed = false;
    }

    if(1)  // �ж����а������Ѿ������¹�����һ���Ե���3��led��
    {
        static bool keyTestFisrtFinish = false;
        if(!keyTestflag && !keyTestFisrtFinish)
        {
            keyTestFisrtFinish = true;
            HalLedSet(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, HAL_LED_MODE_ON);
        }
    }
    
    // ��������
    if(key_pressed)
    {   
        // ��ʾ������Ϣ�����ȫ�����������¹�����ʾ���԰����ɹ�����ɫ��ʾ
        sprintf(str, "%s ON", key_info2);
        HalLcdWriteString(str, HAL_LCD_LINE_8 | (keyTestflag ? 0 : HAL_LCD_REVERSE));

        sprintf(str, "%s ON", key_info);
    }
    else
    {
        // ��ʾ������Ϣ�����ȫ�����������¹�����ʾ���԰����ɹ�����ɫ��ʾ
        sprintf(str, "%s OFF", key_info2);
        HalLcdWriteString(str, HAL_LCD_LINE_8 | (keyTestflag ? 0 : HAL_LCD_REVERSE));

        sprintf(str, "%s OFF", key_info);
    }

    /*
    ���Ͱ�����Ϣ:
    ���δ�������������ߴӻ�����ͨ�����ڷ��ͳ���
    ������������������ߴӻ��������Ϣ���͵��Զ�
    */
    if(1)
    {
        char buffer[20];
        sprintf(buffer, "%s\r\n", str);
        
        if(!simpleBLE_IfConnected())// δ����
        {
            NPI_WriteTransport((uint8*)buffer, strlen(buffer)); // ͨ�����ڷ��ͳ���
            simpleBLE_Delay_1ms(1);
        }
        else //������
        {         
            qq_write((uint8*)buffer, strlen(buffer));

            // �����¼���Ȼ�����¼�����������ʱ����ʱ������ݲ����͵�����
            osal_set_event( simpleBLETaskId, SBP_UART_EVT );                    
        }
    }
}


// MT ����� ����
bool simpleBLE_MT_CMD_Handle(uint8 *pBuffer, uint16 length)
{
    char strTemp[64];
    uint8 i;

    /*
    ֧����������:
    AT+LED2=0  ��LED2��
    AT+LED2=1  ��LED2��
    AT+LED3=0  ��LED3��
    AT+LED3=1  ��LED3��
    ����LED1��Ϊϵͳָʾ�ƣ�����Ͳ���Ϊ���������ˣ� ���������������
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
        uint8 ch = pBuffer[6];  // AT+ADC��β�ţ�����AT+ADC4�е�4
        uint16 adc;         // adc ����    
        float volt;         // ��ѹֵ
        uint8 channel;      // ͨ��
        
        switch(ch)
        {
            case '4'://����P0.4�˿ڵ�ѹ adc   
                channel = HAL_ADC_CHN_AIN4; goto NEXT_ADC;
            case '5'://����P0.5�˿ڵ�ѹ adc
                channel = HAL_ADC_CHN_AIN5; goto NEXT_ADC;
            case '6'://����P0.6�˿ڵ�ѹ adc
                channel = HAL_ADC_CHN_AIN6; goto NEXT_ADC;
            case '7'://����P0.7�˿ڵ�ѹ adc
                channel = HAL_ADC_CHN_AIN7; goto NEXT_ADC;
    NEXT_ADC:
                HalAdcSetReference( HAL_ADC_REF_AVDD );
                adc = HalAdcRead( channel, HAL_ADC_RESOLUTION_14 ); //HAL_ADC_RESOLUTION_14  ����ЧλΪ13λ���뿴�ú����ڲ�����
                volt = adc*3.3/8192;
                // ��ʽ��
                sprintf(strTemp, "OK+Get:ADC%d=%d,%.2fv\r\n", ch-'0',adc, volt);
    //            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                qq_write((uint8*)strTemp, strlen(strTemp));
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );                 
                break;
            case '8'://���������ѹ
            {
                // �ο��Ա�Э��ջ�е� Battservice.c �ļ�
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
                    // �ٷֱ�
                    percent = (uint8) ((((adc - battMinLevel) * 25) + (range - 1)) / range);
                }
                // ��ʽ��
                sprintf(strTemp, "OK+Get:ADC%d=%d,%d%%\r\n", ch-'0',adc, percent);
    //            NPI_WriteTransport((uint8*)strTemp, osal_strlen(strTemp)); 
                qq_write((uint8*)strTemp, strlen(strTemp));
                osal_set_event( simpleBLETaskId, SBP_UART_EVT );                 
                break;
            }
            case '9'://����оƬ�ڲ��¶�
            {
                /*
                �ο���ַ
                https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/303585
                */
#define TEMP_CONST 0.61065 // (1250 / 2047) 
#define TEMP_OFFSET_DATASHEET 750 
#define TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS 29.75 
#define TEMP_OFFSET (TEMP_OFFSET_DATASHEET + TEMP_OFFSET_MEASURED_AT_25_DEGREES_CELCIUS) // 779.75 
#define TEMP_COEFF 4.5  

                float AvgTemp;   
                float AvgTemp_sum = 0;               

                TR0 = 0x01;                     //����Ϊ1�������¶ȴ�������SOC_ADC
                ATEST = 0x01;                   //ʹ���¶ȴ���            

                // ��64��ƽ��ֵ���Ա��ȶ�
                for(i = 0; i<64; i++)
                {
                    HalAdcSetReference( HAL_ADC_REF_125V );
                    adc = HalAdcRead( HAL_ADC_CHN_TEMP, HAL_ADC_RESOLUTION_12 );
                    AvgTemp = adc * TEMP_CONST; 
                    AvgTemp =  ((AvgTemp - TEMP_OFFSET) / TEMP_COEFF); 
                    AvgTemp_sum += AvgTemp;
                }
                // �õ�ʵ���¶�
                AvgTemp = AvgTemp_sum/64; 
                
                TR0 &= ~0x01;
                ATEST &= ~0x01;
                
                // ��ʽ��
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

#if defined(USE_DISPLAY_KEY_VALUE)  // ���԰���ר�ã���ʾ5�򰴼�ֵ
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
        P0SEL &= ~0x40; // ����P0.6Ϊ���ܽţ�����ʵ���ǻᷢ�ֹܽ��в�������
        P0DIR &= ~0x40;
        adc = HalAdcRead (HAL_ADC_CHANNEL_6, HAL_ADC_RESOLUTION_8); //������7λ���ݣ����ֵ128-1
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
