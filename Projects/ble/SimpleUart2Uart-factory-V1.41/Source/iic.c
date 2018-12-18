/****************************************************************************
* �� �� ��: main.c
* ��    ��: Amo [ www.amoMcu.com ��Ī��Ƭ��]
* ��    ��: 2014-04-08
* ��    ��: 1.0
* ��    ��: IIC io��ģ������
****************************************************************************/

#include <ioCC2540.h>
#include "IIC.h"

#define  uchar unsigned  char 
#define  uint  unsigned  int
void IIC_Init(void)//IIC��ʼ��
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���

    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
}
void Signal_Start(void)//IIC��ʼ�ź�
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���
    SDA = 1;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 0;
    delay_1ms(); 
}
void Signal_Stop(void)//IICֹͣ�ź�
{
    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���
    SDA = 0;
    delay_1ms();
    SCL = 1;
    delay_1ms();
    SDA = 1;
    delay_1ms(); 
}
void Respons(void)//��Ӧ�ź�
{
    uint i = 0;

    P1DIR |= 0x20;      //P1.5����Ϊ���
    P1DIR &= ~0x40;      //P1.6����Ϊ����

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

    P1DIR |= 0x60;      //P1.5��P1.6����Ϊ���
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

    P1DIR |= 0x20;      //P1.5����Ϊ���
    P1DIR &= ~0x40;      //P1.6����Ϊ����

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
    Signal_Start(); //����һ����ʼ�ź�
    Write_Byte(comd);
    Respons();      //�ȴ���Ӧ
    Write_Byte(add);
    Respons();      //�ȴ���Ӧ
    Write_Byte(wdata);
    Respons();      //�ȴ���Ӧ
    Signal_Stop();  //����һ����ֹ�ź�
}
uchar Read_Add(uchar add,uchar comd)
{
    uchar tdata;
    Signal_Start();     //����һ����ʼ�ź�
    Write_Byte(comd);
    Respons();          //�ȴ���Ӧ
    Write_Byte(add);
    Respons();          //�ȴ���Ӧ
    Signal_Start();     //�ٲ���һ����ʼ�ź�
    Write_Byte(comd|0x01);
    Respons();          //�ȴ���Ӧ
    tdata = Read_Byte();
    Signal_Stop();      //����һ����ֹ�ź�
    return tdata;       
}
void delay_1ms(void)    //��� 0us  ��ʱ1ms
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
