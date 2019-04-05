//===========================================================================//
//                                                                           //
// �ļ���  FlashDrv.c                                                        //
// ˵����  flash����                                                         //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "FlashDrv.h"

/*******************************************************************************
// ����: ��FLASH����
// ����: unsigned int waddr 16λ��ַ
         unsigned char *pword_buff ��ŵ�ַ
         unsigned char len ��ȡ����
// ���: unsigned char ����һ���ֽ�����
*******************************************************************************/
void FLASH_ReadPage(unsigned int waddr,unsigned char *pword_buff,unsigned char len)
{
  while(FCTL3 & BUSY);
  while(len--)
  {
    *pword_buff++ = *(unsigned char *)waddr++;
  }
}