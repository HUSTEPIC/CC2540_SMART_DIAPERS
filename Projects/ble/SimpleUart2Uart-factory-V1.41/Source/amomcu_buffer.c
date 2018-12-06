#include "bcomdef.h"
#include "stdio.h"
#include "string.h"



#if 1
//以下函数实现fifo的写读个功能
// 参考自
// http://blog.csdn.net/shoutday/article/details/8204770
// 修改---amomcu---------2015.08.31-----用在uart上非常方便---------


#define SEND_BUF_MAX_SIZE   320

static char sendBufForUsart[SEND_BUF_MAX_SIZE] = {0};        //usart send buffer.
static unsigned short sendWrite = 0;                          //usart send buffer write position.
static unsigned short sendRead = 0;                           //usart send buffer read position.


//写  WrLen 个字节数据到 缓冲区 RdBuf， 返回 true 表示成功， 返回false表示剩余缓冲区放不下这段数据了
bool qq_write(uint8 *WrBuf, unsigned short WrLen)
{
    unsigned short emptyLen;
    unsigned short tmpAddr;
    unsigned short tmpLen;

    emptyLen = (sendRead+SEND_BUF_MAX_SIZE-(sendWrite+1)) % SEND_BUF_MAX_SIZE;
    if (emptyLen >= WrLen)
    {
         tmpAddr = (sendWrite+WrLen) % SEND_BUF_MAX_SIZE;
         if (tmpAddr <= sendWrite)            //If Circular array have inverse to begin.
         {
              tmpLen =WrLen - tmpAddr;
              memcpy(&sendBufForUsart[sendWrite], WrBuf, tmpLen);   //bug place
              memcpy(&sendBufForUsart[0], WrBuf+tmpLen, tmpAddr);    
         }
         else
         {
              memcpy(&sendBufForUsart[sendWrite], WrBuf, WrLen);
         }
         
         sendWrite = tmpAddr;

         return TRUE;
    }

    return FALSE;
}


// 读  RdLen 个字节数据到 缓冲区 RdBuf， 返回读取到的有效数据长度
unsigned short qq_read(uint8 *RdBuf, unsigned short RdLen)
{
     unsigned short validLen;
     unsigned short tmpAddr;
     unsigned short tmpLen;

     validLen = (sendWrite+SEND_BUF_MAX_SIZE-sendRead) % SEND_BUF_MAX_SIZE;

     if(validLen == 0)
        return 0;
     
     if (validLen < RdLen)
        RdLen = validLen;
     
     if (validLen >= RdLen)
     {
         tmpAddr = (sendRead+RdLen) % SEND_BUF_MAX_SIZE;
         if (tmpAddr <= sendRead) //If Circular array have inverse to begin.
         {
           tmpLen =RdLen - tmpAddr;
           memcpy(RdBuf, &sendBufForUsart[sendRead], tmpLen);
           memcpy(RdBuf+tmpLen, &sendBufForUsart[0], tmpAddr);     
         }
         else
         {
           memcpy(RdBuf, &sendBufForUsart[sendRead], RdLen);
         }
         sendRead = tmpAddr;

    }

    return RdLen;
}

// 读出缓冲区中有效数据的大小，一般用于判断有没有数据可读
unsigned short qq_total()
{
    unsigned short validLen;

    validLen = (sendWrite+SEND_BUF_MAX_SIZE-sendRead) % SEND_BUF_MAX_SIZE;

    return validLen;
}

// 清除缓冲区
void qq_clear()
{
    sendWrite = 0;                          //usart send buffer write position.
    sendRead = 0;                           //usart send buffer read position.
}


#endif

/*********************************************************************
*********************************************************************/
