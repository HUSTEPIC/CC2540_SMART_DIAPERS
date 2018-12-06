#include "bcomdef.h"
#include "stdio.h"
#include "string.h"



#if 1
//���º���ʵ��fifo��д��������
// �ο���
// http://blog.csdn.net/shoutday/article/details/8204770
// �޸�---amomcu---------2015.08.31-----����uart�Ϸǳ�����---------


#define SEND_BUF_MAX_SIZE   320

static char sendBufForUsart[SEND_BUF_MAX_SIZE] = {0};        //usart send buffer.
static unsigned short sendWrite = 0;                          //usart send buffer write position.
static unsigned short sendRead = 0;                           //usart send buffer read position.


//д  WrLen ���ֽ����ݵ� ������ RdBuf�� ���� true ��ʾ�ɹ��� ����false��ʾʣ�໺�����Ų������������
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


// ��  RdLen ���ֽ����ݵ� ������ RdBuf�� ���ض�ȡ������Ч���ݳ���
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

// ��������������Ч���ݵĴ�С��һ�������ж���û�����ݿɶ�
unsigned short qq_total()
{
    unsigned short validLen;

    validLen = (sendWrite+SEND_BUF_MAX_SIZE-sendRead) % SEND_BUF_MAX_SIZE;

    return validLen;
}

// ���������
void qq_clear()
{
    sendWrite = 0;                          //usart send buffer write position.
    sendRead = 0;                           //usart send buffer read position.
}


#endif

/*********************************************************************
*********************************************************************/
