//===========================================================================//
//                                                                           //
// �ļ���  eeprom.h                                                          //
// ˵����  AT24C256��д����                                                  //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.5                        //
// �汾��  v1.1 
// ʱ�䣺  2017/03/03                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUSR                                                             //
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