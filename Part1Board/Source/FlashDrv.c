//===========================================================================//
//                                                                           //
// 文件：  FlashDrv.c                                                        //
// 说明：  flash驱动                                                         //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "FlashDrv.h"

/*******************************************************************************
// 描述: 读FLASH操作
// 输入: unsigned int waddr 16位地址
         unsigned char *pword_buff 存放地址
         unsigned char len 读取长度
// 输出: unsigned char 返回一个字节数据
*******************************************************************************/
void FLASH_ReadPage(unsigned int waddr,unsigned char *pword_buff,unsigned char len)
{
  while(FCTL3 & BUSY);
  while(len--)
  {
    *pword_buff++ = *(unsigned char *)waddr++;
  }
}