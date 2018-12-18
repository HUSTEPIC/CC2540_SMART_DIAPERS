/*************************************************************************************************************
Բ�㲩ʿС���������2014������Դ��������:
��Դ��������ο�,Բ�㲩ʿ����Դ�����ṩ�κ���ʽ�ĵ���,Ҳ������ʹ�ø�Դ��������ֵ���ʧ����.
�û�������ѧϰ��Ŀ���޸ĺ�ʹ�ø�Դ����.
���û����޸ĸ�Դ����ʱ,�����Ƴ��ò��ְ�Ȩ��Ϣ�����뱣��ԭ������.

������Ϣ������ʹٷ���վwww.etootle.com, �ٷ�����:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include <ioCC2540.h>
#include "gpio_i2c.h"

void ANBT_I2C_Configuration(void)			
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

	ANBT_I2C_SCL_1; 
	ANBT_I2C_SDA_1;
	ANBT_I2C_DELAY;
}

void ANBT_I2C_Delay(uint32 dly)               
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

uint8 ANBT_I2C_START(void)
{ 
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

	ANBT_I2C_SDA_1; 
 	ANBT_I2C_NOP;
  // 
 	ANBT_I2C_SCL_1; 
 	ANBT_I2C_NOP;    
	//
 	if(!ANBT_I2C_SDA_STATE) return ANBT_I2C_BUS_BUSY;
	//
 	ANBT_I2C_SDA_0;
 	ANBT_I2C_NOP;
  //
 	ANBT_I2C_SCL_0;  
 	ANBT_I2C_NOP; 
	//
 	if(ANBT_I2C_SDA_STATE) return ANBT_I2C_BUS_ERROR;
	//
 	return ANBT_I2C_READY;
}

void ANBT_I2C_STOP(void)
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

 	ANBT_I2C_SDA_0; 
 	ANBT_I2C_NOP;
  // 
 	ANBT_I2C_SCL_1; 
 	ANBT_I2C_NOP;    
	//
 	ANBT_I2C_SDA_1;
 	ANBT_I2C_NOP;
}

void ANBT_I2C_SendACK(void)
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

 	ANBT_I2C_SDA_0;
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_1;
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_0; 
 	ANBT_I2C_NOP;  
}

void ANBT_I2C_SendNACK(void)
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

	ANBT_I2C_SDA_1;
	ANBT_I2C_NOP;
	ANBT_I2C_SCL_1;
	ANBT_I2C_NOP;
	ANBT_I2C_SCL_0; 
	ANBT_I2C_NOP;  
}

uint8 ANBT_I2C_SendByte(uint8 anbt_i2c_data)
{
 	uint8 i;
 	
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

	ANBT_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(anbt_i2c_data&0x80) ANBT_I2C_SDA_1;
   		else ANBT_I2C_SDA_0;
			//
  		anbt_i2c_data<<=1;
  		ANBT_I2C_NOP;
			//
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP;
  		ANBT_I2C_SCL_0;
  		ANBT_I2C_NOP; 
 	}
	//
 	ANBT_I2C_SDA_1; 
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_1;
 	ANBT_I2C_NOP;   
 	if(ANBT_I2C_SDA_STATE)
 	{
  		ANBT_I2C_SCL_0;
  		return ANBT_I2C_NACK;
 	}
 	else
 	{
  		ANBT_I2C_SCL_0;
  		return ANBT_I2C_ACK;  
 	}    
}

uint8 ANBT_I2C_ReceiveByte(void)
{
	uint8 i,anbt_i2c_data;
	//
    P1DIR |= 0x20;      //P1.5����Ϊ���
    P1DIR &= ~0x40;      //P1.6����Ϊ����

 	ANBT_I2C_SDA_1;
 	ANBT_I2C_SCL_0; 
 	anbt_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
 	}
	ANBT_I2C_SendNACK();
 	return anbt_i2c_data;
}

uint8 ANBT_I2C_ReceiveByte_WithACK(void)
{
	uint8 i,anbt_i2c_data;
	//
    P1DIR |= 0x20;      //P1.5����Ϊ���
    P1DIR &= ~0x40;      //P1.6����Ϊ����

 	ANBT_I2C_SDA_1;
 	ANBT_I2C_SCL_0; 
 	anbt_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
 	}
	ANBT_I2C_SendACK();
 	return anbt_i2c_data;
}

void ANBT_I2C_Receive6Bytes(uint8 *anbt_i2c_data_buffer)
{
	uint8 i,j;
	uint8 anbt_i2c_data;

	for(j=0;j<5;j++)
	{
        P1DIR |= 0x20;      //P1.5����Ϊ���
        P1DIR &= ~0x40;      //P1.6����Ϊ����

		ANBT_I2C_SDA_1;
		ANBT_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		ANBT_I2C_SendACK();
	}
	//
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

	ANBT_I2C_SDA_1;
	ANBT_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	ANBT_I2C_SCL_1;
  	ANBT_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	ANBT_I2C_SCL_0;  
  	ANBT_I2C_NOP;         
	}
	anbt_i2c_data_buffer[5]=anbt_i2c_data;
	ANBT_I2C_SendNACK();
}

void ANBT_I2C_Receive12Bytes(uint8 *anbt_i2c_data_buffer)
{
	uint8 i,j;
	uint8 anbt_i2c_data;

	for(j=0;j<11;j++)
	{
        P1DIR |= 0x20;      //P1.5����Ϊ���
        P1DIR &= ~0x40;      //P1.6����Ϊ����

		ANBT_I2C_SDA_1;
		ANBT_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		ANBT_I2C_SendACK();
	}
	//
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

	ANBT_I2C_SDA_1;
	ANBT_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	ANBT_I2C_SCL_1;
  	ANBT_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	ANBT_I2C_SCL_0;  
  	ANBT_I2C_NOP;         
	}
	anbt_i2c_data_buffer[11]=anbt_i2c_data;
	ANBT_I2C_SendNACK();
}

void ANBT_I2C_Receive14Bytes(uint8 *anbt_i2c_data_buffer)
{
	uint8 i,j;
	uint8 anbt_i2c_data;

	for(j=0;j<13;j++)
	{
        P1DIR |= 0x20;      //P1.5����Ϊ���
        P1DIR &= ~0x40;      //P1.6����Ϊ����

		ANBT_I2C_SDA_1;
		ANBT_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		ANBT_I2C_SCL_1;
  		ANBT_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		ANBT_I2C_SCL_0;  
  		ANBT_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		ANBT_I2C_SendACK();
	}
	//
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���
	ANBT_I2C_SDA_1;
	ANBT_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
        P1DIR |= 0x20;      //P1.5����Ϊ���
        P1DIR &= ~0x40;      //P1.6����Ϊ����
  	ANBT_I2C_SCL_1;
  	ANBT_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(ANBT_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	ANBT_I2C_SCL_0;  
  	ANBT_I2C_NOP;         
	}
	anbt_i2c_data_buffer[13]=anbt_i2c_data;
	ANBT_I2C_SendNACK();
}

void AnBT_DMP_Delay_us(uint32 dly)
{
	uint8 i;
	while(dly--) for(i=0;i<10;i++);
}
//
void AnBT_DMP_Delay_ms(uint32 dly)
{
	while(dly--) AnBT_DMP_Delay_us(1000);
}
//

uint8 AnBT_DMP_I2C_Write(uint8 anbt_dev_addr, uint8 anbt_reg_addr, uint8 anbt_i2c_len, uint8 *anbt_i2c_data_buf)
{		
		uint8 i;
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | I2C_Direction_Transmitter);					//Բ�㲩ʿ:����������д��ַ
		ANBT_I2C_SendByte(anbt_reg_addr);  //Բ�㲩ʿ:����������PWM��ַ
		for (i=0;i<anbt_i2c_len;i++) ANBT_I2C_SendByte(anbt_i2c_data_buf[i]); //Բ�㲩ʿ:����������PWMֵ
		ANBT_I2C_STOP();
		return 0x00;
}
uint8 AnBT_DMP_I2C_Read(uint8 anbt_dev_addr, uint8 anbt_reg_addr, uint8 anbt_i2c_len, uint8 *anbt_i2c_data_buf)
{
	
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | I2C_Direction_Transmitter);			//Բ�㲩ʿ:����������д��ַ
		ANBT_I2C_SendByte(anbt_reg_addr);  //Բ�㲩ʿ:����������ID��ַ
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | I2C_Direction_Receiver);      //Բ�㲩ʿ:���������Ƕ���ַ
		//
    while (anbt_i2c_len)
		{
			if (anbt_i2c_len==1) *anbt_i2c_data_buf =ANBT_I2C_ReceiveByte();  
      else *anbt_i2c_data_buf =ANBT_I2C_ReceiveByte_WithACK();
      anbt_i2c_data_buf++;
      anbt_i2c_len--;
    }
		ANBT_I2C_STOP();
    return 0x00;
}
