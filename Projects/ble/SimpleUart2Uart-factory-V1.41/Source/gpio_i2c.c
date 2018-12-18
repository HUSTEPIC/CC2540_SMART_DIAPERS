/*************************************************************************************************************
圆点博士小四轴飞行器2014版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include <ioCC2540.h>
#include "gpio_i2c.h"

void ANBT_I2C_Configuration(void)			
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

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
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

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
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

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
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

 	ANBT_I2C_SDA_0;
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_1;
 	ANBT_I2C_NOP;
 	ANBT_I2C_SCL_0; 
 	ANBT_I2C_NOP;  
}

void ANBT_I2C_SendNACK(void)
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

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
 	
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

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
    P1DIR |= 0x20;      //P1.5定义为输出
    P1DIR &= ~0x40;      //P1.6定义为输入

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
    P1DIR |= 0x20;      //P1.5定义为输出
    P1DIR &= ~0x40;      //P1.6定义为输入

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
        P1DIR |= 0x20;      //P1.5定义为输出
        P1DIR &= ~0x40;      //P1.6定义为输入

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
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

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
        P1DIR |= 0x20;      //P1.5定义为输出
        P1DIR &= ~0x40;      //P1.6定义为输入

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
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

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
        P1DIR |= 0x20;      //P1.5定义为输出
        P1DIR &= ~0x40;      //P1.6定义为输入

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
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出
	ANBT_I2C_SDA_1;
	ANBT_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
        P1DIR |= 0x20;      //P1.5定义为输出
        P1DIR &= ~0x40;      //P1.6定义为输入
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
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | I2C_Direction_Transmitter);					//圆点博士:发送陀螺仪写地址
		ANBT_I2C_SendByte(anbt_reg_addr);  //圆点博士:发送陀螺仪PWM地址
		for (i=0;i<anbt_i2c_len;i++) ANBT_I2C_SendByte(anbt_i2c_data_buf[i]); //圆点博士:发送陀螺仪PWM值
		ANBT_I2C_STOP();
		return 0x00;
}
uint8 AnBT_DMP_I2C_Read(uint8 anbt_dev_addr, uint8 anbt_reg_addr, uint8 anbt_i2c_len, uint8 *anbt_i2c_data_buf)
{
	
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | I2C_Direction_Transmitter);			//圆点博士:发送陀螺仪写地址
		ANBT_I2C_SendByte(anbt_reg_addr);  //圆点博士:发送陀螺仪ID地址
		ANBT_I2C_START();
		ANBT_I2C_SendByte(anbt_dev_addr << 1 | I2C_Direction_Receiver);      //圆点博士:发送陀螺仪读地址
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
