//===========================================================================//
//                                                                           //
// 文件：  FlashDrv.h                                                        //
// 说明：  flash驱动                                                         //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#ifndef FlashDrv_H
#define FlashDrv_H
void FLASH_ReadPage(unsigned int waddr,unsigned char *pword_buff,unsigned char len);
#endif

