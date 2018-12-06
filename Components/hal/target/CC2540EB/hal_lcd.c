/*
Oled0.96 显示屏驱动程序 基于SSD1306驱动芯片，128x64 点阵显示.

阿莫智能设备淘宝店   https://amomcu.taobao.com/
阿莫开源社区论坛     www.amomcu.com

2017.01.06

显示屏购买地址:
  https://item.taobao.com/item.htm?spm=a1z10.1-c-s.w4004-15079397444.11.OuENIp&id=38157235836

该显示屏配套本店的3款开发板, 只要1个或2个显示屏, 其他开发板都适用 :
[1]:  cc2540 / cc2541 蓝牙4.0LE 开发板 :
  https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15079397460.14.6RWrSk&id=38116104998

[2]:  cc2640 / cc2650 蓝牙4.1 蓝牙4.2 LE 开发板 :
  https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15079397460.25.lHqIhg&id=521167677080

[3]:  cc3200 低功耗wifi 音视频传输开发板 :
  https://item.taobao.com/item.htm?spm=a1z10.5-c-s.w4002-15079397460.22.pjsTjA&id=527067675797

专注做有技术支持的蓝牙与wifi开发板，谢谢支持 阿莫智能设备淘宝店 !
*/

#include "hal_types.h"
#include "hal_lcd.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "stdio.h"

#if (HAL_LCD == TRUE)

#if (HAL_UART == TRUE)
#ifdef LCD_TO_UART      // 串口输出显示的内容
  #include "npi.h"
#endif
#endif

/*
注意 : 该驱动中，我们已经方便于你修改程序了， 
如下面的 LCD_SPI_SCLK_SBIT 为P_5,
如果你想修改为P0_0, 则修改如下:
#define LCD_SPI_SCLK_SBIT         P0_0       //SCLK  时钟 D0（SCLK）
#define LCD_SPI_SCLK_DDR          P0DIR      //端口
#define LCD_SPI_SCLK_BV           BV(0)      //位(0~7)
*/

/*
本显示屏线序如下(左到右):
1-----------GND  =  接地
2-----------VCC  =  电源一般2V~5V
3-----------D0   =  SPI_SCLK
4-----------D1   =  SPI_MOSI
5-----------RST  =  SPI_RESET
6-----------DC   =  SPI_DC
本显示屏使用spi接口，只有输出没有输入，并且cs线没有引出，已默认选通，
dc线本不属于spi接口范畴，但是该显示屏有这个额外的数据与命令选择线，
dc为1时，通过spi接口写入的是显示数据
dc为0时，通过spi接口写入的是命令.
*/

// oled显示屏上的 D0 线
#define LCD_SPI_SCLK_SBIT         P1_5       //SCLK  时钟 D0（SCLK）
#define LCD_SPI_SCLK_DDR          P1DIR      //端口
#define LCD_SPI_SCLK_BV           BV(5)      //位(范围0~7)

// oled显示屏上的 D1 线
#define LCD_SPI_MOSI_SBIT         P1_6       //SDA   D1（MOSI） 数据
#define LCD_SPI_MOSI_DDR          P1DIR      //端口
#define LCD_SPI_MOSI_BV           BV(6)      //位(范围0~7)

// oled显示屏上的 RST 线
#define LCD_SPI_RESET_SBIT        P1_7       //_RES  hardware reset 复位 
#define LCD_SPI_RESET_DDR         P1DIR      //端口
#define LCD_SPI_RESET_BV          BV(7)      //位(范围0~7)

// oled显示屏上的 DC 线
#define LCD_SPI_DC_SBIT           P1_2       //A0  H/L 命令数据选通端，H：数据，L:命令
#define LCD_SPI_DC_DDR            P1DIR      //端口
#define LCD_SPI_DC_BV             BV(2)      //位(范围0~7)

// 初始化这4个io口为输出
#define LCD_SPI_PORT_INIT()                   \
    do{                                         \
        LCD_SPI_SCLK_DDR |= LCD_SPI_SCLK_BV;    \
        LCD_SPI_MOSI_DDR |= LCD_SPI_MOSI_BV;    \
        LCD_SPI_RESET_DDR |= LCD_SPI_RESET_BV;  \
        LCD_SPI_DC_DDR |= LCD_SPI_DC_BV;        \
   }while(0)

// 定义LCD 的宽高点阵大小
#define X_WIDTH        128      // 宽度点数
#define Y_WIDTH        64       // 高度点数

// 定义LCD 显示用的格式化数组长度，方便后面统一使用该长度
#define LCD_MAX_BUF                     25

// 6*8点阵 的ascii码所有的定义，是用字模2 软件生成的
static const unsigned char F6x8[92][6] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// sp
    0x00, 0x00, 0x00, 0x2f, 0x00, 0x00,// !
    0x00, 0x00, 0x07, 0x00, 0x07, 0x00,// "
    0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14,// #
    0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12,// $
    0x00, 0x62, 0x64, 0x08, 0x13, 0x23,// %
    0x00, 0x36, 0x49, 0x55, 0x22, 0x50,// &
    0x00, 0x00, 0x05, 0x03, 0x00, 0x00,// '
    0x00, 0x00, 0x1c, 0x22, 0x41, 0x00,// (
    0x00, 0x00, 0x41, 0x22, 0x1c, 0x00,// )
    0x00, 0x14, 0x08, 0x3E, 0x08, 0x14,// *
    0x00, 0x08, 0x08, 0x3E, 0x08, 0x08,// +
    0x00, 0x00, 0x00, 0xA0, 0x60, 0x00,// ,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08,// -
    0x00, 0x00, 0x60, 0x60, 0x00, 0x00,// .
    0x00, 0x20, 0x10, 0x08, 0x04, 0x02,// /
    0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
    0x00, 0x00, 0x42, 0x7F, 0x40, 0x00,// 1
    0x00, 0x42, 0x61, 0x51, 0x49, 0x46,// 2
    0x00, 0x21, 0x41, 0x45, 0x4B, 0x31,// 3
    0x00, 0x18, 0x14, 0x12, 0x7F, 0x10,// 4
    0x00, 0x27, 0x45, 0x45, 0x45, 0x39,// 5
    0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
    0x00, 0x01, 0x71, 0x09, 0x05, 0x03,// 7
    0x00, 0x36, 0x49, 0x49, 0x49, 0x36,// 8
    0x00, 0x06, 0x49, 0x49, 0x29, 0x1E,// 9
    0x00, 0x00, 0x36, 0x36, 0x00, 0x00,// :
    0x00, 0x00, 0x56, 0x36, 0x00, 0x00,// ;
    0x00, 0x08, 0x14, 0x22, 0x41, 0x00,// <
    0x00, 0x14, 0x14, 0x14, 0x14, 0x14,// =
    0x00, 0x00, 0x41, 0x22, 0x14, 0x08,// >
    0x00, 0x02, 0x01, 0x51, 0x09, 0x06,// ?
    0x00, 0x32, 0x49, 0x59, 0x51, 0x3E,// @
    0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C,// A
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x36,// B
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x22,// C
    0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C,// D
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x41,// E
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x01,// F
    0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A,// G
    0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F,// H
    0x00, 0x00, 0x41, 0x7F, 0x41, 0x00,// I
    0x00, 0x20, 0x40, 0x41, 0x3F, 0x01,// J
    0x00, 0x7F, 0x08, 0x14, 0x22, 0x41,// K
    0x00, 0x7F, 0x40, 0x40, 0x40, 0x40,// L
    0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F,// M
    0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F,// N
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E,// O
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x06,// P
    0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
    0x00, 0x7F, 0x09, 0x19, 0x29, 0x46,// R
    0x00, 0x46, 0x49, 0x49, 0x49, 0x31,// S
    0x00, 0x01, 0x01, 0x7F, 0x01, 0x01,// T
    0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F,// U
    0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F,// V
    0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F,// W
    0x00, 0x63, 0x14, 0x08, 0x14, 0x63,// X
    0x00, 0x07, 0x08, 0x70, 0x08, 0x07,// Y
    0x00, 0x61, 0x51, 0x49, 0x45, 0x43,// Z
    0x00, 0x00, 0x7F, 0x41, 0x41, 0x00,// [
    0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55,// 55
    0x00, 0x00, 0x41, 0x41, 0x7F, 0x00,// ]
    0x00, 0x04, 0x02, 0x01, 0x02, 0x04,// ^
    0x00, 0x40, 0x40, 0x40, 0x40, 0x40,// _
    0x00, 0x00, 0x01, 0x02, 0x04, 0x00,// '
    0x00, 0x20, 0x54, 0x54, 0x54, 0x78,// a
    0x00, 0x7F, 0x48, 0x44, 0x44, 0x38,// b
    0x00, 0x38, 0x44, 0x44, 0x44, 0x20,// c
    0x00, 0x38, 0x44, 0x44, 0x48, 0x7F,// d
    0x00, 0x38, 0x54, 0x54, 0x54, 0x18,// e
    0x00, 0x08, 0x7E, 0x09, 0x01, 0x02,// f
    0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,// g
    0x00, 0x7F, 0x08, 0x04, 0x04, 0x78,// h
    0x00, 0x00, 0x44, 0x7D, 0x40, 0x00,// i
    0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,// j
    0x00, 0x7F, 0x10, 0x28, 0x44, 0x00,// k
    0x00, 0x00, 0x41, 0x7F, 0x40, 0x00,// l
    0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,// m
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x78,// n
    0x00, 0x38, 0x44, 0x44, 0x44, 0x38,// o
    0x00, 0xFC, 0x24, 0x24, 0x24, 0x18,// p
    0x00, 0x18, 0x24, 0x24, 0x18, 0xFC,// q
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x08,// r
    0x00, 0x48, 0x54, 0x54, 0x54, 0x20,// s
    0x00, 0x04, 0x3F, 0x44, 0x40, 0x20,// t
    0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C,// u
    0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C,// v
    0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C,// w
    0x00, 0x44, 0x28, 0x10, 0x28, 0x44,// x
    0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,// y
    0x00, 0x44, 0x64, 0x54, 0x4C, 0x44,// z
    0x14, 0x14, 0x14, 0x14, 0x14, 0x14,// horiz lines
};

uint8 static g_reverseflag = 0;

// 延时1ms 的倍数
void LCD_DLY_ms(unsigned int ms)
{                         
    unsigned int a;
    while(ms)
    {
        a=1800;
        while(a--);
        ms--;
    }
}

// LCD写一个字节数据
void LCD_WrDat(unsigned char dat)     
{
    unsigned char i=8, temp=0;
    LCD_SPI_DC_SBIT = 1;  
    for(i=0;i<8;i++) //发送一个八位数据 
    {
        LCD_SPI_SCLK_SBIT = 0;  
        
        temp = dat&0x80;
        if (temp == 0)
        {
            LCD_SPI_MOSI_SBIT = 0;
        }
        else
        {
            LCD_SPI_MOSI_SBIT = 1;
        }
        LCD_SPI_SCLK_SBIT = 1;             
        dat<<=1;    
    }
}

// LCD写一个字节命令
void LCD_WrCmd(unsigned char cmd)
{
    unsigned char i=8, temp=0;
    LCD_SPI_DC_SBIT=0;
    for(i=0;i<8;i++) //发送一个八位数据 
    { 
        LCD_SPI_SCLK_SBIT=0; 
       
        temp = cmd&0x80;
        if (temp == 0)
        {
            LCD_SPI_MOSI_SBIT = 0;
        }
        else
        {
            LCD_SPI_MOSI_SBIT = 1;
        }
        LCD_SPI_SCLK_SBIT=1;
        cmd<<=1;;        
    }     
}

// LCD 设置坐标
void LCD_Set_Pos(unsigned char x, unsigned char y) 
{ 
    LCD_WrCmd(0xb0+y);
    LCD_WrCmd(((x&0xf0)>>4)|0x10);
    LCD_WrCmd((x&0x0f)|0x01); 
} 

// LCD全屏填充同一个数据，实现全屏显示同一种颜色，一般为全部点亮或者灭屏
void LCD_Fill(unsigned char bmp_dat) 
{
    unsigned char y,x;
    for(y=0;y<8;y++)
    {
        LCD_WrCmd(0xb0+y);
        LCD_WrCmd(0x01);
        LCD_WrCmd(0x10);
        for(x=0;x<X_WIDTH;x++)
            LCD_WrDat(bmp_dat);
    }
}

// 功能描述：行填充, y为页范围0～7
void LCD_FillLine(unsigned char y,unsigned char ch)
{
    unsigned char x;    
    LCD_WrCmd(0xb0+y);
    LCD_WrCmd(0x01);
    LCD_WrCmd(0x10); 
    for(x=0;x<X_WIDTH;x++)
        LCD_WrDat(ch);
}

// LCD初始化， 通过spi接口写数据到oled显示屏， oled显示屏控制器为 SSD1306
void HalLcdInit(void)
{
    LCD_SPI_PORT_INIT();  // 初始化这4个io口为输出

    LCD_SPI_SCLK_SBIT=0;
    LCD_SPI_MOSI_SBIT=0;
    LCD_SPI_DC_SBIT=0;
    LCD_SPI_RESET_SBIT=0;    
    LCD_DLY_ms(10);
    LCD_SPI_RESET_SBIT=1; //从上电到下面开始初始化要有足够的时间，即等待RC复位完毕   
    LCD_DLY_ms(10);
    LCD_WrCmd(0xae);//--turn off oled panel
    LCD_WrCmd(0x00);//---set low column address
    LCD_WrCmd(0x10);//---set high column address
    LCD_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    LCD_WrCmd(0x81);//--set contrast control register
    LCD_WrCmd(0xcf); // Set SEG Output Current Brightness
    LCD_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    LCD_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    LCD_WrCmd(0xa6);//--set normal display
    LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
    LCD_WrCmd(0x3f);//--1/64 duty
    LCD_WrCmd(0xd3);//-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
    LCD_WrCmd(0x00);//-not offset
    LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
    LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    LCD_WrCmd(0xd9);//--set pre-charge period
    LCD_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    LCD_WrCmd(0xda);//--set com pins hardware configuration
    LCD_WrCmd(0x12);
    LCD_WrCmd(0xdb);//--set vcomh
    LCD_WrCmd(0x40);//Set VCOM Deselect Level
    LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    LCD_WrCmd(0x02);//
    LCD_WrCmd(0x8d);//--set Charge Pump enable/disable
    LCD_WrCmd(0x14);//--set(0x10) disable
    LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
    LCD_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7) 
    LCD_WrCmd(0xaf);//--turn on oled panel
    LCD_Fill(0x00); //初始成黑屏
    LCD_Set_Pos(0,0);   
} 

// 显示6*8一组标准ASCII字符串    显示的坐标（x,y），y为页范围0～7
void LCD_P6x8Str(unsigned char x, unsigned char y,unsigned char ch[])
{
    unsigned char c=0,i=0,j=0;      
    while (ch[j]!='\0')
    {    
        c =ch[j]-32;
        if(x>126){x=0;y++;}
        LCD_Set_Pos(x,y);  
        if(c > 0 && c < 92)
        {
            if(g_reverseflag)
            {
                for(i=0;i<6;i++)     
                    LCD_WrDat(~F6x8[c][i]);  
            }
            else
            {
                for(i=0;i<6;i++)     
                    LCD_WrDat(F6x8[c][i]);  
            }
        }
        x+=6;
        j++;
    }
}

// 显示一行ascii 码，无显示的点阵显示黑色,     line为页的范围1～8 
void HalLcd_HW_WriteLine(unsigned char line, const char *pText)
{
    if(g_reverseflag)
    {
        LCD_FillLine(line-1, 0xFF);          //先把这一行显示成反色-就是点亮
    }
    else
    {
        LCD_FillLine(line-1, 0);             //先把这一行显示成黑色
    }
    LCD_P6x8Str(0, line-1, (unsigned char *)pText);
}

// 显示一行ascii 码， option 为页的范围1～8  或取反显示(按摩添加的)
void HalLcdWriteString ( char *str, uint8 option)
{
  uint8 strLen = 0;
  uint8 totalLen = 0;
  uint8 *buf;
  uint8 tmpLen;
  static uint8 *Lcd_Line1 = NULL;

  g_reverseflag = (option & HAL_LCD_REVERSE);
  option &= (~HAL_LCD_REVERSE);  //去掉最高位
  
#if (HAL_UART == TRUE)
#ifdef LCD_TO_UART      // 串口输出显示的内容
  NPI_WriteTransport ( (uint8*)str,osal_strlen(str)); 
  NPI_WriteTransport ("\r\n",2);
#endif
#endif

  if ( Lcd_Line1 == NULL )
  {
    Lcd_Line1 = osal_mem_alloc( HAL_LCD_MAX_CHARS+1 );
    HalLcdWriteString( "TexasInstruments", 1 );
  }

  strLen = (uint8)osal_strlen( (char*)str );

  /* Check boundries */
  if ( strLen > HAL_LCD_MAX_CHARS )
    strLen = HAL_LCD_MAX_CHARS;

  if ( option == HAL_LCD_LINE_1 )
  {
    /* Line 1 gets saved for later */
    osal_memcpy( Lcd_Line1, str, strLen );
    Lcd_Line1[strLen] = '\0';
  }
  else
  {
    /* Line 2 triggers action */
    tmpLen = (uint8)osal_strlen( (char*)Lcd_Line1 );
    totalLen =  tmpLen + 1 + strLen + 1;
    buf = osal_mem_alloc( totalLen );
    if ( buf != NULL )
    {
      /* Concatenate strings */
      osal_memcpy( buf, Lcd_Line1, tmpLen );
      buf[tmpLen++] = ' ';
      osal_memcpy( &buf[tmpLen], str, strLen );
      buf[tmpLen+strLen] = '\0';
      
      /* Free mem */
      osal_mem_free( buf );
    }
  }

  /* Display the string */
  HalLcd_HW_WriteLine (option, str);

  g_reverseflag &= (~HAL_LCD_REVERSE);
}

/* 显示1个10或16进制数据 
   value  : 显示数值
   radix  : 显示格式，10表示10进制显示，或者16表示16进制显示 
   option : 为页的范围1～8 
*/   
void HalLcdWriteValue ( uint32 value, const uint8 radix, uint8 option)
{
  uint8 buf[LCD_MAX_BUF];

  _ltoa( value, &buf[0], radix );
  HalLcdWriteString( (char*)buf, option );
}

/*
 * Write a value to the LCD
 */
void HalLcdWriteScreen( char *line1, char *line2 )
{
  HalLcdWriteString( line1, 1 );
  HalLcdWriteString( line2, 2 );
}

/* 显示1个带标题的10或16进制数据 
   title  : 标题
   value  : 显示数值
   format : 显示格式，10表示10进制显示，或者16表示16进制显示 
   option : 为页的范围1～8 
*/   
void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line )
{
  uint8 tmpLen;
  uint8 buf[LCD_MAX_BUF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  HalLcdWriteString( (char*)buf, line );		
}

/* 显示2个带标题的10或16进制数据 
   title   : 标题
   value1  : 显示数值
   format1 : 显示格式，10表示10进制显示，或者16表示16进制显示 
   value2  : 显示数值
   format2 : 显示格式，10表示10进制显示，或者16表示16进制显示 
   line    : 为页的范围1～8 
*/   
void HalLcdWriteStringValueValue( char *title, uint16 value1, uint8 format1,
                                  uint16 value2, uint8 format2, uint8 line )
{
  uint8 tmpLen;
  uint8 buf[LCD_MAX_BUF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  if ( tmpLen )
  {
    osal_memcpy( buf, title, tmpLen );
    buf[tmpLen++] = ' ';
  }

  err = (uint32)(value1);
  _ltoa( err, &buf[tmpLen], format1 );
  tmpLen = (uint8)osal_strlen( (char*)buf );

  buf[tmpLen++] = ',';
  buf[tmpLen++] = ' ';
  err = (uint32)(value2);
  _ltoa( err, &buf[tmpLen], format2 );

  HalLcdWriteString( (char *)buf, line );	
}

/* 显示1个带标题的百分比数据
   title   : 标题
   value   : 显示数值
*/   
void HalLcdDisplayPercentBar( char *title, uint8 value )
{
  uint8 percent;
  uint8 leftOver;
  uint8 buf[17];
  uint32 err;
  uint8 x;

  /* Write the title: */
  HalLcdWriteString( title, HAL_LCD_LINE_1 );

  if ( value > 100 )
    value = 100;

  /* convert to blocks */
  percent = (uint8)(value / 10);
  leftOver = (uint8)(value % 10);

  /* Make window */
  osal_memcpy( buf, "[          ]  ", 15 );

  for ( x = 0; x < percent; x ++ )
  {
    buf[1+x] = '>';
  }

  if ( leftOver >= 5 )
    buf[1+x] = '+';

  err = (uint32)value;
  _ltoa( err, (uint8*)&buf[13], 10 );

  HalLcdWriteString( (char*)buf, HAL_LCD_LINE_2 );
}
#else
/*
 * Initialize LCD Service
 */
void HalLcdInit(void){}
/*
 * Write a string to the LCD
 */
extern void HalLcdWriteString ( char *str, uint8 option){}

/*
 * Write a value to the LCD
 */
extern void HalLcdWriteValue ( uint32 value, const uint8 radix, uint8 option){}

/*
 * Write a value to the LCD
 */
extern void HalLcdWriteScreen( char *line1, char *line2 ){}

/*
 * Write a string followed by a value to the LCD
 */
extern void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line ){}

/*
 * Write a string followed by 2 values to the LCD
 */
extern void HalLcdWriteStringValueValue( char *title, uint16 value1, uint8 format1, uint16 value2, uint8 format2, uint8 line ){}

/*
 * Write a percentage bar to the LCD
 */
extern void HalLcdDisplayPercentBar( char *title, uint8 value ){}
#endif

