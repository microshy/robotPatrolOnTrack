//===========================================================================//
//                                                                           //
// �ļ���  FlashDrv.h                                                        //
// ˵����  flash����                                                         //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//
#ifndef FlashDrv_H
#define FlashDrv_H
void FLASH_ReadPage(unsigned int waddr,unsigned char *pword_buff,unsigned char len);
#endif

