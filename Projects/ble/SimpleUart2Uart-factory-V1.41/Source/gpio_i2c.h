/*****************************************************************************************
Բ�㲩ʿSTM32����4.0��Сϵͳ��AnBT���ļ�������Դ���롣��Բ�㲩ʿSTM32������������������ݡ�
����������LGPLЭ�顣�û���������ת��ʹ�øô��룬�������Ƴ��ò��ְ�Ȩ��Ϣ
������Ϣ������ʹٷ���վwww.etootle.com
******************************************************************************************/

//#include "usart.h"
//
//#define uint32 unsigned long
//#define u16 unsigned short
//#define uint8   unsigned char
#include "hal_types.h"


#define  I2C_Direction_Transmitter      ((uint8)0x00)
#define  I2C_Direction_Receiver         ((uint8)0x01)
#define IS_I2C_DIRECTION(DIRECTION) (((DIRECTION) == I2C_Direction_Transmitter) || \
                                     ((DIRECTION) == I2C_Direction_Receiver))


//#define AnBT_MPU6050_INT			1//GPIO_Pin_3		//PB3
//#define AnBT_MPU6050_INT_PORT	//GPIOB
//
//#define ANBT_MPU6050_INT_STATE   GPIO_ReadInputDataBit(AnBT_MPU6050_INT_PORT, AnBT_MPU6050_INT)
//
#if 1
#define SCL P1_5                //����ģ��IIC��ʱ����
#define SDA P1_6                //����ģ��IIC��������
#else
#define SCL P0_7                //����
#define SDA P0_6                //����
#endif



#define ANBT_I2C_SDA 		SDA	 
#define ANBT_I2C_SCL 		SCL	
//#define ANBT_I2C_PORT   GPIOB
//
#define ANBT_I2C_SCL_0 		SCL=0
#define ANBT_I2C_SCL_1 		SCL=1
#define ANBT_I2C_SDA_0 		SDA=0
#define ANBT_I2C_SDA_1   	SDA=1
//
#define ANBT_I2C_SDA_STATE   	SDA
#define ANBT_I2C_DELAY 				ANBT_I2C_Delay(100000)
#define ANBT_I2C_NOP					ANBT_I2C_Delay(1) //ANBT_I2C_Delay(10) 
//
#define ANBT_I2C_READY		0x00
#define ANBT_I2C_BUS_BUSY	0x01	
#define ANBT_I2C_BUS_ERROR	0x02
//
#define ANBT_I2C_NACK	  0x00 
#define ANBT_I2C_ACK		0x01

// I2C acknowledge
typedef enum{
  ACK  = 0,
  NACK = 1,
}etI2cAck;

//
void ANBT_I2C_Configuration(void);
void ANBT_I2C_Delay(uint32 dly);
uint8 ANBT_I2C_START(void);
void ANBT_I2C_STOP(void);
void ANBT_I2C_SendACK(void);
void ANBT_I2C_SendNACK(void);
uint8 ANBT_I2C_SendByte(uint8 anbt_i2c_data);
uint8 ANBT_I2C_ReceiveByte_WithACK(void);
uint8 ANBT_I2C_ReceiveByte(void);
void ANBT_I2C_Receive12Bytes(uint8 *anbt_i2c_data_buffer);
void ANBT_I2C_Receive6Bytes(uint8 *anbt_i2c_data_buffer);
void ANBT_I2C_Receive14Bytes(uint8 *anbt_i2c_data_buffer);
uint8 AnBT_DMP_I2C_Write(uint8 anbt_dev_addr, uint8 anbt_reg_addr, uint8 anbt_i2c_len, uint8 *anbt_i2c_data_buf);
uint8 AnBT_DMP_I2C_Read(uint8 anbt_dev_addr, uint8 anbt_reg_addr, uint8 anbt_i2c_len, uint8 *anbt_i2c_data_buf);
void AnBT_DMP_Delay_us(uint32 dly);
void AnBT_DMP_Delay_ms(uint32 dly);



