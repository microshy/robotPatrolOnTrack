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

void ASCII2HEX(void);
void DoDataExg(void);

void FLASH_EraseSEG(unsigned int *Address);
void FLASH_Writew(unsigned int *Address,unsigned int nValue);
unsigned char FLASH_Read(unsigned int waddr);
void FLASH_ReadPage(unsigned int waddr,unsigned char *pword_buff,unsigned char len);
void FLASH_EraseBOK(void);

void FlashMaster(unsigned char temp);
void FLASH_EraseBOK(void);
void Flash_Program(void);

void FlashMaster(unsigned char temp);
void FlashUpdate();
#endif

