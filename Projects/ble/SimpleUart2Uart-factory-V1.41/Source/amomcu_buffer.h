

#ifndef AMOMCU_BUFFER_H
#define AMOMCU_BUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "hal_types.h"

//д  WrLen ���ֽ����ݵ� ������ RdBuf�� ���� true ��ʾ�ɹ��� ����false��ʾʣ�໺�����Ų������������
extern bool qq_write(uint8 *WrBuf, unsigned short WrLen);

// ��  RdLen ���ֽ����ݵ� ������ RdBuf�� ���ض�ȡ������Ч���ݳ���
extern unsigned short qq_read(uint8 *RdBuf, unsigned short RdLen);

// ��������������Ч���ݵĴ�С��һ�������ж���û�����ݿɶ�
extern unsigned short qq_total();

// ���������
extern void qq_clear();





#ifdef __cplusplus
}
#endif

#endif
