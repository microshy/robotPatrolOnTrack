//===========================================================================//
//                                                                           //
// 文件：  Uart.c                                                            //
// 说明：  串口模块                                                          //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//

#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "Uart.h"
#include "eeprom.h"
unsigned char CRC8Table[] =
{ 
    //0
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E, 
    //1
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    //2
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    //3
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    //4 
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    //5
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    //6
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    //7
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    //8
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    //9
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    //A
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    //B
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    //C
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    //D
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xFE, 0x79, 0x48, 0x1B, 0x2A,
    //E
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    //F
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC

};
/*******************************************************************************
Uart0相关的数据保存，O2传感器返回值
*******************************************************************************/
unsigned char Uart0RXBuffer[1];
unsigned char Uart0RXFlag=0X00;//数据接收标志
extern unsigned char Uart0InstructFlag=0x00;//接收到指令标志
unsigned char Uart0DataBuffer[10];//数据保存数组
unsigned char Uart0DataNum=0;//数据保存计数
extern unsigned char O2Sensor[4]={0x00};

/*******************************************************************************
Uart1相关的数据保存，485总线
*******************************************************************************/
unsigned char Uart1RXBuffer[1];
unsigned char Uart1RXFlag=0X00;//数据接收标志
unsigned char Uart1DataBuffer[260]={0x00};//数据保存数组
int Uart1DataBfLength=0;//数据保存长度
int Uart1DataNum=0;//数据保存计数
extern unsigned char Uart1InstructFlag=0x00;//接收到指令标志
unsigned char Uart1InstructSource=0x00;//指令发送源地址
unsigned char Uart1InstructAim=0x00;//指令发送目的地址
extern unsigned char Uart1InstructNum=0x00;//指令编号

extern unsigned char obstacle_status=0x00;//障碍物情况
extern unsigned char Uart1moveRes[4]={0x00};//move指令相关信息
extern unsigned char Uart1movetoResX[4]={0x00};//moveto指令X相关信息
extern unsigned char Uart1movetoResY[2]={0x00};//moveto指令Y相关信息

extern unsigned char Uart1UpdateID=0x00;//rom更新设备ID
extern unsigned char Uart1UpdateDataNum=0x00;//rom更新数据序号
unsigned char UpdateDataNum;
extern unsigned char Uart1UpdateStatus=0x00;//rom更新状态位
extern unsigned int addrEeprom=0x0000;//
extern unsigned char updateState=0x00;//软件更新状态
unsigned char dataLost=0x00;
/*******************************************************************************
Uart2相关的数据保存，485 小车编码器返回值 
*******************************************************************************/
unsigned char Uart2RXBuffer[1];
unsigned char Uart2RXFlag=0X00;//数据接收标志
unsigned char Uart2DataBuffer[20]={0x00};
extern unsigned char Uart2InstructFlag=0x00;//接收到指令标志
int Uart2DataNum=0;
extern long LencoderX=0;
extern long LencoderXOld=0;
extern long LlocationX =0;
/*******************************************************************************
Uart3相关的数据保存, 485 伸缩杆编码器返回值
*******************************************************************************/
unsigned char Uart3RXBuffer[1];
unsigned char Uart3RXFlag=0X00;//数据接收标志
unsigned char Uart3DataBuffer[20]={0x00};//数据保存数组
int Uart3DataNum=0;//数据保存计数
extern unsigned char Uart3InstructFlag=0x00;//接收到指令标志
extern long LencoderY=0;
extern long LencoderYOld=0;
extern long LlocationY =0;

//****************************************************************************
//                                                                           *
//                       串口初始化                                          *
//                                                                           *
//****************************************************************************
//---------------------------串口1 485总线-115200-----------------------------//
void Init_UART1(void)
{
    UCA1CTL1  = UCSWRST                                        ; // 状态机复位
    P5SEL |= TXD485BUS + RXD485BUS                                ;  //引脚功能选择
    P5DIR |= TXD485BUS                                           ;  //设置方向输出
    P5DIR &= ~ RXD485BUS                                        ;  //设置方向输入
    
    UCA1CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA1BR0   = 0x45                                           ; // 8MHz/115200=69 
    UCA1BR1   = 0x00                                           ; 
    UCA1MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
        
    UCA1CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA1IE    |= UCRXIE                                         ; // 允许接收中断
    
    UART1_IN;

}
//-------------------------串口2 小车编码器串口-19200------------------------//
void Init_UART2(void)
{
    UCA2CTL1  = UCSWRST                                        ; // 状态机复位
    P9SEL |= TXD485_ENCODER1  + RXD485_ENCODER1                               ;  //引脚功能选择
    P9DIR |= TXD485_ENCODER1                                           ;  //设置方向输出
    P9DIR &= ~ RXD485_ENCODER1                                        ;  //设置方向输入
	
    UCA2CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA2BR0   = 0xA0                                           ; // 8MHz/19200=416 
    UCA2BR1   = 0x01                                           ; 
    UCA2MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=6, UCBRFx=0
    
    UCA2CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA2IE    |= UCRXIE                                         ; // 允许接收中断
		
    UART2_IN;
}
//------------------------串口3 伸缩杆编码器串口-19200-----------------------//
void Init_UART3(void)
{
    UCA3CTL1  = UCSWRST                                        ; // 状态机复位
    P10SEL |= TXD485_ENCODER2  + RXD485_ENCODER2                               ;  //引脚功能选择
    P10DIR |= TXD485_ENCODER2                                           ;  //设置方向输出
    P10DIR &= ~ RXD485_ENCODER2                                         ;  //设置方向输入

    UCA3CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA3BR0   = 0xA0                                           ; // 8MHz/19200=416 
    UCA3BR1   = 0x01                                           ; 
    UCA3MCTL  = UCBRS_6 + UCBRF_0                              ; // UCBRSx=6, UCBRFx=0
    
    UCA3CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA3IE    |= UCRXIE                                         ; // 允许接收中断
    
    UART3_IN;
}
/*******************************************************************************
UART1模块的数据发送 485 总线
*******************************************************************************/
void Uart1_send(unsigned char *tx_buf,unsigned char length)
{
  unsigned char i;
  UART1_OUT;
  for(i=0;i<length;i++)
  {
    UCA1TXBUF = *tx_buf++ ; 
    while (!(UCA1IFG&UCTXIFG)) ; 
  }
  __delay_cycles(8000) ;
  UART1_IN ;
}
/*******************************************************************************
UART2模块的数据发送 485 小车编码器
*******************************************************************************/
void Uart2_send(unsigned char *tx_buf,unsigned char length)
{
  unsigned char i ;
  UART2_OUT;
  for(i=0;i<length;i++)
  {
    UCA2TXBUF = *tx_buf++ ; 
    while (!(UCA2IFG&UCTXIFG)) ; 
  }
  __delay_cycles(8000) ;
  UART2_IN ;
}
/*******************************************************************************
UART3模块的数据发送 485 伸缩杆编码器
*******************************************************************************/
void Uart3_send(unsigned char *tx_buf,unsigned char length)
{
  UART3_OUT;
  unsigned char i ;
  for(i=0;i<length;i++)
  {
    UCA3TXBUF = *tx_buf++ ; 
    while (!(UCA3IFG&UCTXIFG)) ; 
  }
  __delay_cycles(8000) ;
  UART3_IN;
}

/*******************************************************************************
UART1模块的数据接收 485总线
*******************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    Uart1RXBuffer[0]=0x00;
    switch(__even_in_range(UCA1IV,4))
    {
        case 0:break ; // Vector 0 - no interrupt
        case 2:   // Vector 2 - RXIFG
            Uart1RXBuffer[0] = UCA1RXBUF ;
            switch (Uart1RXFlag)
            {
            case 0x00://数据头
              if(Uart1RXBuffer[0]==0xFE)
              {
                  Uart1RXFlag=0x01;
              }
              break;
            case 0x01://数据长度位
              Uart1DataBfLength=Uart1RXBuffer[0];//记录数据长度
              Uart1DataNum=0;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//保存数据长度
              Uart1RXFlag=0x02;//找到数据长度位
              break;
            case 0x02://目的地址
              Uart1DataNum++;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//保存目的地址
              Uart1InstructAim=Uart1RXBuffer[0];
              Uart1RXFlag=0x03;//找到数据长度位
              break;
            case 0x03://源地址
              Uart1DataNum++;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//保存源地址
              Uart1InstructSource=Uart1RXBuffer[0];
              Uart1RXFlag=0x04;//找到数据长度位
              break;
            case 0x04://指令编号
              Uart1DataNum++;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//保存指令编号
              Uart1InstructNum=Uart1RXBuffer[0];
              Uart1RXFlag=0x05;//找到数据长度位
              break;
            case 0x05://数据位和CRC位
              Uart1DataNum++;//用于计数
              Uart1DataBuffer[Uart1DataNum]=Uart1RXBuffer[0];//数据位
              if(Uart1DataBfLength<=Uart1DataNum)//接收预计数量的数据
              {
                unsigned char crc=Uart1RXBuffer[0];//保存CRC位
                if((Uart1InstructAim==0x04) &&(crc == CRC(Uart1DataBuffer,Uart1DataBfLength)))//目的地址正确，校验正确
                {
                  Uart1InstructFlag=0x01;
                  Uart1InstructNum=Uart1ExtrInstr(Uart1DataBuffer ,Uart1DataBfLength);
                  
                  if(Uart1InstructNum==0x18)
                  {
                      if(Uart1UpdateDataNum==0x00)//第一帧
                      {
                          updateState=0x01;
                          dataLost=0x00;//
                          UpdateDataNum=Uart1UpdateDataNum;
                          eeprom_write_page(0x0000+(unsigned int)(Uart1UpdateDataNum*249),Uart1DataBuffer+6 ,(unsigned char)(Uart1DataBfLength-6));
                      }
                      if(Uart1UpdateDataNum!=0x00)//如果不是第一帧
                      {
                          if(UpdateDataNum+1==Uart1UpdateDataNum)//如果是连续帧，写eeprom
                          {
                            UpdateDataNum=Uart1UpdateDataNum;
                            eeprom_write_page(0x0000+(unsigned int)(Uart1UpdateDataNum*249),Uart1DataBuffer+6 ,(unsigned char)(Uart1DataBfLength-6));
                          }
                          else if(UpdateDataNum==Uart1UpdateDataNum)//重复帧
                          {
                            UpdateDataNum=Uart1UpdateDataNum;
                          }
                          else
                          {
                            dataLost=0x01;//数据丢失
                          }
                      }
                      __delay_cycles(8000);
                      unsigned char buffer_1[5]={0x05 ,0x01 ,0x05, 0x18, Uart1UpdateDataNum};
                      unsigned char crc=CRC(buffer_1,5);
                      unsigned char buffer[7]={0xFE , 0x05 ,0x01,0x05, 0x18 , Uart1UpdateDataNum , crc};
                      Uart1_send(buffer,7);
                      
                      if(Uart1UpdateStatus==0x01)//最后一帧，分两种情况，如果程序包完全接受正确，则可以正常更新，否则不能更新程序，继续执行旧的程序
                      {
                        if(dataLost==0x00)
                        {
                            delay_ms(5);
                            unsigned char c=0x55;
                            eeprom_writepage(0xFFFF,&c,1);//写标志，告诉引导代码有现成的程序，可以直接更新
                            
                            UCA1IE &= ~UCRXIE ; // 关闭接收中断
                            UCA2IE &= ~UCRXIE ; // 关闭接收中断
                            UCA3IE &= ~UCRXIE ; // 关闭接收中断
                            TB0CCTL0 &= ~CCIE ; // 关闭时钟中断
                            P2IE &= ~JJ1 ;
                            WDTCTL = 0;
                            //((void (*)())0xFFFE)();
                        }
                        else
                          updateState=0x00;//跳过更新
                      }
                  }
                }
                else
                {
                  Uart1InstructFlag=0x00;
                }
                Uart1RXFlag=0x00;
              }
              break;
            default :
              Uart1RXFlag=0x00;
              break;
              
            }
            break ;
        case 4:break ;  // Vector 4 - TXIFG
        default: break ;  
    }  
}

/*******************************************************************************
串口2数据接收模块 485 小车编码器
串口2连接绝对式编码器，编码器读数据时的指令格式是
发送：D + 地址 + 0X0D
接收：X + 地址 + > + 符合位 + 数据位 + 0X0D
需要在接收中断中判断正确数据，并保存有用数据
*******************************************************************************/
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
    switch(__even_in_range(UCA2IV,4))
    {
        case 0:break ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart2RXBuffer[0] = UCA2RXBUF ;      
            
            switch(Uart2RXFlag)
            {
            case 0x00:
              if((Uart2RXBuffer[0]=='X')||(Uart2RXBuffer[0]==0x58))//'X'
              {
                Uart2RXFlag=0x01;
                Uart2DataNum=0;
              }
              Uart2DataNum=0;
              Uart2InstructFlag=0x00;//接收失败
              break;
            case 0x01:
              if(Uart2RXBuffer[0]==0x30)//地址位
              {
                Uart2RXFlag=0x02;
                Uart2DataNum=0;
                break;
              }
              Uart2InstructFlag=0x00;//接收失败
              Uart2RXFlag=0x00;
              Uart2DataNum=0;
              break;
            case 0x02:
              if(Uart2RXBuffer[0]==0x30)//地址位
              {
                Uart2RXFlag=0x03;
                Uart2DataNum=0;
                break;
              }
              Uart2InstructFlag=0x00;//接收失败
              Uart2RXFlag=0x00;
              Uart2DataNum=0;
              break;
            case 0x03://'>'
              if((Uart2RXBuffer[0]=='>'))//分割位
              {
                Uart2RXFlag=0x04;
                Uart2DataNum=0;
                break;
              }
              if(Uart2RXBuffer[0]==0x6A)
              {
                Uart2RXFlag=0x05;
                Uart2DataNum=0;
                break;
              }
              Uart2InstructFlag=0x00;//接收失败
              Uart2RXFlag=0x00;
              Uart2DataNum=0;
              break;
            case 0x04://符号位
              Uart2RXFlag=0x05;
              Uart2DataNum=0;
              break;
            case 0x05://数据位10位
              
              Uart2DataBuffer[Uart2DataNum]=Uart2RXBuffer[0];
              Uart2DataNum++;
              if(Uart2DataNum>9)
              {
                  Uart2RXFlag=0x06;
              }
              break;
            case 0x06://数据尾
              if(Uart2RXBuffer[0]==0x0D)//地址位
              {
                Uart2InstructFlag=0x01;//接收成功
                LencoderX=((long)(Uart2DataBuffer[0]-48))*1000000000
                       +((long)(Uart2DataBuffer[1]-48))*100000000
                         +((long)(Uart2DataBuffer[2]-48))*10000000
                           +((long)(Uart2DataBuffer[3]-48))*1000000
                             +((long)(Uart2DataBuffer[4]-48))*100000
                               +((long)(Uart2DataBuffer[5]-48))*10000
                                 +((long)(Uart2DataBuffer[6]-48))*1000
                                   +((long)(Uart2DataBuffer[7]-48))*100
                                     +((long)(Uart2DataBuffer[8]-48))*10
                                       +((long)(Uart2DataBuffer[9]-48))*1;
                
                LlocationX=(LencoderX-1000000)/28;//转换成位置值，都保存下来，需要时直接用
                Uart2RXFlag=0x00;
                Uart2DataNum=0;
                break;
              }
              Uart2InstructFlag=0x00;//接收成功
              Uart2RXFlag=0x00;
              Uart2DataNum=0;
              break;
            }
            break ;
        case 4:break ;  // Vector 4 - TXIFG
        default: break ;  
    }
}
/*******************************************************************************
串口3数据接收模块 485 伸缩杆编码器
*******************************************************************************/
#pragma vector=USCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
{
    
    Uart3RXBuffer[0]=0x00;
    switch(__even_in_range(UCA3IV,4))
    {
        case 0:break ; // Vector 0 - no interrupt
        case 2:   // Vector 2 - RXIFG
            Uart3RXBuffer[0] = UCA3RXBUF ;      
            
            switch(Uart3RXFlag)
            {
            case 0x00:
              if(Uart3RXBuffer[0]=='X')//'X'
              {
                Uart3RXFlag=0x01;
                Uart3DataNum=0;
              }
              Uart3DataNum=0;
              Uart3InstructFlag=0x00;//接收失败
              break;
            case 0x01:
              if(Uart3RXBuffer[0]==0x30)//地址位
              {
                Uart3RXFlag=0x02;
                Uart3DataNum=0;
                break;
              }
              Uart3InstructFlag=0x00;//接收失败
              Uart3RXFlag=0x00;
              Uart3DataNum=0;
              break;
            case 0x02:
              if(Uart3RXBuffer[0]==0x30)//地址位
              {
                Uart3RXFlag=0x03;
                Uart3DataNum=0;
                break;
              }
              Uart3InstructFlag=0x00;//接收失败
              Uart3RXFlag=0x00;
              Uart3DataNum=0;
              break;
            case 0x03://'>'
              if(Uart3RXBuffer[0]=='>')//地址位
              {
                Uart3RXFlag=0x04;
                Uart3DataNum=0;
                break;
              }
              if(Uart3RXBuffer[0]==0x6A)
              {
                Uart3RXFlag=0x05;
                Uart3DataNum=0;
                break;
              }
              Uart3InstructFlag=0x00;//接收失败
              Uart3RXFlag=0x00;
              Uart3DataNum=0;
              break;
            case 0x04://符号位
              Uart3RXFlag=0x05;
              Uart3DataNum=0;
              break;
            case 0x05://数据位10位
              
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];
              Uart3DataNum++;
              if(Uart3DataNum>=10)
              {
                  Uart3RXFlag=0x06;
              }
              break;
            case 0x06://数据尾
              if(Uart3RXBuffer[0]==0x0D)//地址位
              {
                Uart3InstructFlag=0x01;//接收成功
                LencoderY=((long)(Uart3DataBuffer[0]-48))*1000000000
                       +((long)(Uart3DataBuffer[1]-48))*100000000
                         +((long)(Uart3DataBuffer[2]-48))*10000000
                           +((long)(Uart3DataBuffer[3]-48))*1000000
                             +((long)(Uart3DataBuffer[4]-48))*100000
                               +((long)(Uart3DataBuffer[5]-48))*10000
                                 +((long)(Uart3DataBuffer[6]-48))*1000
                                   +((long)(Uart3DataBuffer[7]-48))*100
                                     +((long)(Uart3DataBuffer[8]-48))*10
                                       +((long)(Uart3DataBuffer[9]-48))*1;
                
                LlocationY=(LencoderY-1000000)/17;//转换成位置值，都保存下来，需要时直接用
                Uart3RXFlag=0x00;
                Uart3DataNum=0;
                break;
              }
              Uart3InstructFlag=0x00;//接收成功
              Uart3RXFlag=0x00;
              Uart3DataNum=0;
              break;
            }
            break ;
        case 4:break ;  // Vector 4 - TXIFG
        default: break ;  
    }  
  
}

/*******************************************************************************
8位CRC
计算CRC校验位
*******************************************************************************/
unsigned char CRC(unsigned char *buffer, int length)
{
    unsigned char crc = 0x00;
    for (int i = 0; i < length; i++)
    {
        crc = CRC8Table[crc ^ buffer[i]];
    }
    return crc;
}
/******************************************************
*函数名称:CRC16RTU
*输   入:pszBuf  要校验的数据
        unLength 校验数据的长
*输   出:校验值
*功   能:循环冗余校验-16
         （RTU标准-0xA001）
*******************************************************/
unsigned int CRC16RTU( unsigned char * pszBuf, unsigned int unLength)
{
    unsigned int CRC=0XFFFF;
    unsigned long CRC_count;
    for(CRC_count=0;CRC_count<unLength;CRC_count++)
    {
        int i;
        CRC=CRC^*(pszBuf+CRC_count);

        for(i=0;i<8;i++)
        {
            if(CRC&1)
            {
                CRC>>=1;
                CRC^=0xA001;
            }
            else
            { 
                CRC>>=1;
            }
        }
    }
    return CRC;
}
/*******************************************************************************
对子控制板传来的指令进行提取
*******************************************************************************/

unsigned char Uart1ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[3])
    {
        case 0x01://状态查询指令
            obstacle_status=buffer[4];//障碍物状态
            //obstacle_status=0x00;
            return 0x01;
            
        case 0x02://小车或伸缩杆运动指令
          for(int i=0;i<4;i++)
            Uart1moveRes[i]=buffer[4+i];
          
            return 0x02;
                    
        case 0x03://小车或伸缩杆停止指令
            return 0x03;

        case 0x04://小车运动到指定位置指令
          for(int i=0;i<4;i++)
            Uart1movetoResX[i]=buffer[4+i];
            return 0x04;

        case 0x05://小车移动到位完成指令
            return 0x05;

        case 0x06://伸缩杆移动到位指令
          for(int i=0;i<2;i++)
            Uart1movetoResY[i]=buffer[4+i];
            return 0x06;

        case 0x07://伸缩杆移动到位完成指令
            return 0x07;
            
        case 0x08://复位指令
            return 0x08;

        case 0x18://软件更新
          //Uart3UpdateID=buffer[2];//设备ID
          Uart1UpdateDataNum=buffer[4];//数据序号
          Uart1UpdateStatus=buffer[5];//状态位
            return 0x18;
        default:
            return 0x00;//表示提取数据失败
    }
}





