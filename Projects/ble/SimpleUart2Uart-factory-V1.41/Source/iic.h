#ifndef _IIC_H_
#define _IIC_H_

/***ucahr和uint 的宏定义很重要，否则下面的函数无法正常运行******/

#define uchar unsigned char   //定义uchar型数据为无符号型
#define uint unsigned int     //定义uint型数据为无符号型

#if 1
#define SCL P1_5                //定义模拟IIC的时钟线
#define SDA P1_6                //定义模拟IIC的数据线
#else
#define SCL P0_7                //定义
#define SDA P0_6                //定义
#endif

/***申明外部函数****/
extern void delay_1ms(void);
extern void IIC_Init(void);                  //IIC初始化
extern void Signal_Start(void);              //IIC停止信号
extern void Signal_Stop(void);              //IIC停止信号
extern void Write_Byte(uchar wdata);        //写一个字节数据函数
extern uchar Read_Byte();                  //读一个字节数据函数
extern void Write_Add(uchar add,uchar wdata,uchar comd);    //向某个IIC器件写指令，地址和数据
extern uchar Read_Add(uchar add,uchar comd);               //向某个IIC器件写指令读某个地址里面的数据

#endif
