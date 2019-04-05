//===========================================================================//
//                                                                           //
// 文件：  eeprom.h                                                          //
// 说明：  AT24C256读写程序                                                  //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.5                        //
// 版本：  v1.1 
// 时间：  2017/03/03                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUSR                                                             //
//                                                                           //
//===========================================================================//
#ifndef eeprom_H
#define eeprom_H

void Init_EEPROM();
unsigned char eeprom_writebyte( unsigned int Iword_addr , unsigned char word_value );
unsigned char eeprom_writepage( unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len);
unsigned int eeprom_write_page( unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len);
unsigned char eeprom_readbyte( unsigned int Iword_addr , unsigned char *pword_value );
unsigned char eeprom_readpage(unsigned int Iword_addr , unsigned char *pword_buf , unsigned char len );

#endif