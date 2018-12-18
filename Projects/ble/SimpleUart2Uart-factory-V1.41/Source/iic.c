/****************************************************************************
* 文 件 名: main.c
* 作    者: Amo [ www.amoMcu.com 阿莫单片机]
* 修    订: 2014-04-08
* 版    本: 1.0
* 描    述: IIC io口模拟驱动
****************************************************************************/

#include <ioCC2540.h>
#include "IIC.h"

#define  uchar unsigned  char 
#define  uint  unsigned  int
void IIC_Init(void)//IIC初始化
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出

    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
}
void Signal_Start(void)//IIC起始信号
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出
    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 0;
    delay_1ms(); 
}
void Signal_Stop(void)//IIC停止信号
{
    P1DIR |= 0x60;      //P1.5、P1.6定义为输出
    SDA = 0;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 1;
    delay_1ms(); 
}
void Respons(void)//答应信号
{
    uint i = 0;

    P1DIR |= 0x20;      //P1.5定义为输出
    P1DIR &= ~0x40;      //P1.6定义为输入

    SCL = 1;
    delay_1ms();
    SCL = 0;
    delay_1ms();
    
    if(i>=300)
    {
      delay_1ms();
    }
}
void Write_Byte(uchar wdata)
{
    uchar i,mdata;

    P1DIR |= 0x60;      //P1.5、P1.6定义为输出
    mdata = wdata;
    for(i=0;i<8;i++)
    {        
        SCL = 0;
        delay_1ms();
        if(mdata & 0x80)
        {
          SDA = 1;
        }
        else
        {
          SDA = 0;
        }
        delay_1ms();
        SCL = 1;
        delay_1ms();  
        mdata <<= 1;
    }
    SCL = 0;
    delay_1ms();
    SCL = 1;
    delay_1ms();
}
uchar Read_Byte()
{
    uchar i,rdata = 0;

    P1DIR |= 0x20;      //P1.5定义为输出
    P1DIR &= ~0x40;      //P1.6定义为输入

    SCL = 0;
    delay_1ms();
    SCL = 1;
    for(i=0;i<8;i++)
    {
        SCL = 1;
        delay_1ms();
        rdata = (rdata<<1)|SDA;
        SCL = 0;
        delay_1ms();
    }
    return rdata;
}
void Write_Add(uchar add,uchar wdata,uchar comd)
{
    Signal_Start(); //产生一个起始信号
    Write_Byte(comd);
    Respons();      //等待答应
    Write_Byte(add);
    Respons();      //等待答应
    Write_Byte(wdata);
    Respons();      //等待答应
    Signal_Stop();  //产生一个终止信号
}
uchar Read_Add(uchar add,uchar comd)
{
    uchar tdata;
    Signal_Start();     //产生一个起始信号
    Write_Byte(comd);
    Respons();          //等待答应
    Write_Byte(add);
    Respons();          //等待答应
    Signal_Start();     //再产生一个起始信号
    Write_Byte(comd|0x01);
    Respons();          //等待答应
    tdata = Read_Byte();
    Signal_Stop();      //产生一个终止信号
    return tdata;       
}
void delay_1ms(void)    //误差 0us  延时1ms
{
    uchar a,b,c;    
    for(c=4;c>0;c--)
    {
        //for(b=142;b>0;b--)
        {
            for(a=2;a>0;a--)
            {
            }
        }
    }
}
