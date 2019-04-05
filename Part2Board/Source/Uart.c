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
Uart1相关的数据保存，SF6传感器返回值
*******************************************************************************/
unsigned char Uart1RXBuffer[1];
unsigned char Uart1spaceFlag = 0;//判断接收到的字符是不是0x20
unsigned char Uart1FrameHeadFlag = 0;//判断是否接收到（0x20 x020）
unsigned char Uart1DataBuffer[20] ;//存放接收到的数据，不包括数据头（0x20 0x20），包括数据尾（0x0a）
int Uart1DataBufferLength=0;//记录存入dateBufferUart1[100]中的数据长度
int Uart1DataNum = 0;//Uart1接收计数
extern unsigned char SF6Sensor[5]={0x00};//存放SF6浓度数据
extern int SF6_FailCount;
/*******************************************************************************
Uart2相关的数据保存，声音传感器 
*******************************************************************************/
unsigned char Uart2RXBuffer[1];
unsigned char Uart2RXFlag=0X00;//数据接收标志
unsigned char Uart2DataBuffer[7]={0x00};
extern unsigned char Uart2InstructFlag=0x00;//接收到指令标志
int Uart2DataNum=0;
extern unsigned char volumeSensor[2]={0x00};
/*******************************************************************************
Uart3相关的数据保存,485总线
*******************************************************************************/
unsigned char Uart3RXBuffer[1];
unsigned char Uart3RXFlag=0X00;//数据接收标志
unsigned char Uart3DataBuffer[256]={0x00};//数据保存数组
int Uart3DataBfLength=0;//数据保存长度
int Uart3DataNum=0;//数据保存计数
extern unsigned char Uart3InstructFlag=0x00;//接收到指令标志
unsigned char Uart3InstructSource=0x00;//指令发送源地址
unsigned char Uart3InstructAim=0x00;//指令发送目的地址
extern unsigned char Uart3InstructNum=0x00;//指令编号

extern unsigned char updateState=0x00;
unsigned char dataLost=0x00;
unsigned char UpdateDataNum;
extern unsigned char Uart3UpdateStatus=0x00;//rom更新状态位
extern unsigned char Uart3UpdateID=0x00;//rom更新设备ID
extern unsigned char Uart3UpdateDataNum=0x00;//rom更新数据序号
extern unsigned int addrEeprom=0x0000;//
//****************************************************************************
//                                                                           *
//                       串口初始化                                          *
//                                                                           *
//****************************************************************************
//--------------------------串口0 连接O2传感器 9600---------------------------//
void Init_UART0(void)
{
    UCA0CTL1  = UCSWRST ; // 状态机复位
    P3SEL |= TXD_O2 + RXD_O2 ;  //引脚功能选择
    P3DIR |= TXD_O2 ;  //设置方向输出
    P3DIR &= ~ RXD_O2 ;  //设置方向输入
	
    UCA0CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA0BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA0BR1   = 0x03                                           ; 
    UCA0MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
    
    UCA0CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA0IE    |= UCRXIE                                         ; // 允许接收中断

}
//---------------------------串口1 SF6传感器-9600-----------------------------//
void Init_UART1(void)
{
    UCA1CTL1  = UCSWRST                                        ; // 状态机复位
    P5SEL |= TXD_SF6 + RXD_SF6                                ;  //引脚功能选择
    P5DIR |= TXD_SF6                                           ;  //设置方向输出
    P5DIR &= ~ RXD_SF6                                        ;  //设置方向输入
    
    UCA1CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA1BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA1BR1   = 0x03                                           ; 
    UCA1MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
        
    UCA1CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA1IE    |= UCRXIE                                         ; // 允许接收中断

}
//----------------------------串口2 声音传感器--9600--------------------------//
void Init_UART2(void)
{
    UCA2CTL1  = UCSWRST                                        ; // 状态机复位
    P9SEL |= TXD485_VOLUME  + RXD485_VOLUME                               ;  //引脚功能选择
    P9DIR |= TXD485_VOLUME                                           ;  //设置方向输出
    P9DIR &= ~ RXD485_VOLUME                                         ;  //设置方向输入
	
    UCA2CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA2BR0   = 0x41                                           ; // 8MHz/9600=833 
    UCA2BR1   = 0x03                                           ; 
    UCA2MCTL  = UCBRS_2 + UCBRF_0                              ; // UCBRSx=2, UCBRFx=0
    
    UCA2CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA2IE    |= UCRXIE                                         ; // 允许接收中断
		
    UART2_IN;
}
//-----------------------------串口3 485总线--115200----------  --------------//
void Init_UART3(void)
{
    UCA3CTL1  = UCSWRST                                        ; // 状态机复位
    P10SEL |= TXD485BUS  + RXD485BUS                               ;  //引脚功能选择
    P10DIR |= TXD485BUS                                           ;  //设置方向输出
    P10DIR &= ~ RXD485BUS                                         ;  //设置方向输入

    UCA3CTL1  |= UCSSEL_2                                      ; // CLK = SMCLK
    UCA3BR0   = 0x45                                           ; // 8MHz/115200=69 
    UCA3BR1   = 0x00                                           ; 
    UCA3MCTL  = UCBRS_4 + UCBRF_0                              ; // UCBRSx=4, UCBRFx=0
    
    UCA3CTL1  &= ~UCSWRST                                       ; // 启动状态机
    UCA3IE    |= UCRXIE                                         ; // 允许接收中断
    
    UART3_IN;
}
/*******************************************************************************
UART0模块的数据发送
*******************************************************************************/
void Uart0_send(unsigned char *tx_buf,unsigned char length)
{
  unsigned char i ;
  for(i=0;i<length;i++)
  {
    UCA0TXBUF = *tx_buf++ ; 
    while (!(UCA0IFG&UCTXIFG)) ; 
  }
}
/*******************************************************************************
UART1模块的数据发送
*******************************************************************************/
void Uart1_send(unsigned char *tx_buf,unsigned char length)
{
  unsigned char i;
  for(i=0;i<length;i++)
  {
    UCA1TXBUF = *tx_buf++ ; 
    while (!(UCA1IFG&UCTXIFG)) ; 
  }
}
/*******************************************************************************
UART2模块的数据发送 485
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
UART3模块的数据发送 485总线
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
  __delay_cycles(800) ;
  UART3_IN;
}

/*******************************************************************************
UART0模块的数据接收 O2传感器
*******************************************************************************/
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,4))
    {
        case 0:break                                                     ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart0RXBuffer[0] = UCA0RXBUF ;
            
            switch (Uart0RXFlag)
            {
            case 0x00:
                  
                if(Uart0RXBuffer[0]==0x25)//数据头
                {
                    Uart0DataNum=0;//计数
                    Uart0RXFlag=0x01;
                }
                break;
                
            case 0x01:
                if(Uart0RXBuffer[0]!=0x20)
                {
                    Uart0RXFlag=0x00;
                    Uart0InstructFlag = 0;
                    break;
                }
                Uart0RXFlag=0x02;
                break;
            case 0x02:
                
                Uart0DataBuffer[Uart0DataNum]=Uart0RXBuffer[0];
                Uart0DataNum++;//计数
                
                if(Uart0DataNum>5)
                {
                    Uart0RXFlag=0x03;
                    break;
                }
                break;
            case 0x03:
                if(Uart0RXBuffer[0]!=0x0d)//数据尾
                {
                    Uart0RXFlag=0x00;
                    Uart0InstructFlag = 0;
                    break;
                }
                Uart0RXFlag=0x04;
                break;
            case 0x04:
                if(Uart0RXBuffer[0]!=0x0a)//数据尾
                {
                    Uart0RXFlag=0x00;
                    Uart0InstructFlag = 0;
                    break;
                }
                //数据尾正确，表示O2数据接收到
                Uart0RXFlag=0x00;
                Uart0InstructFlag = 0x01;
                
                O2Sensor[0]=Uart0DataBuffer[1];
                O2Sensor[1]=Uart0DataBuffer[2];
                O2Sensor[2]=Uart0DataBuffer[4];
                O2Sensor[3]=Uart0DataBuffer[5];
                break;
                
             default: 
               Uart0InstructFlag=0x00;
               break ;  
              
            }
            break ;
        case 4:break                                                     ;  // Vector 4 - TXIFG
        default: break                                                   ;  
    }  
}
/*******************************************************************************
UART1模块的数据接收 SF6传感器
*******************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV,4))
    {
        case 0:break                                                     ; // Vector 0 - no interrupt
        case 2:                                                            // Vector 2 - RXIFG
            Uart1RXBuffer[0] = UCA1RXBUF ;
            if (Uart1FrameHeadFlag == 1)
            {
                if (Uart1RXBuffer[0]!=0x0a)//否则保存接下来的数据
                {
                    Uart1DataBuffer[Uart1DataNum] = Uart1RXBuffer[0];
                    Uart1DataNum++;
                    if (Uart1DataNum > 20)
                    {
                        Uart1DataNum = 0;
                        Uart1FrameHeadFlag = 0;
                    }
                }
                else //保存完成
                {
                    Uart1DataBuffer[Uart1DataNum] = Uart1RXBuffer[0];
                    Uart1DataBufferLength = Uart1DataNum ;
                    Uart1DataNum = 0;
                    Uart1FrameHeadFlag = 0;
                    SF6_FailCount=0;//出错计数清零
                    if(Uart1DataBufferLength==6)//长度正确，保存数据
                    {
                        SF6Sensor[0]=0x30;
                        SF6Sensor[1]=0x30;
                        SF6Sensor[2]=0x30;
                        SF6Sensor[3]=0x30;
                        SF6Sensor[4]=Uart1DataBuffer[0];
                        break;
                    }
                    else if(Uart1DataBufferLength==7)//长度正确，保存数据
                    {
                        SF6Sensor[0]=0x30;
                        SF6Sensor[1]=0x30;
                        SF6Sensor[2]=0x30;
                        SF6Sensor[3]=Uart1DataBuffer[0];
                        SF6Sensor[4]=Uart1DataBuffer[1];
                        break;
                    }
                    else if(Uart1DataBufferLength==8)//长度正确，保存数据
                    {
                        SF6Sensor[0]=0x30;
                        SF6Sensor[1]=0x30;
                        SF6Sensor[2]=Uart1DataBuffer[0];
                        SF6Sensor[3]=Uart1DataBuffer[1];
                        SF6Sensor[4]=Uart1DataBuffer[2];
                        break;
                    }
                    else if(Uart1DataBufferLength==9)//长度正确，保存数据
                    {
                        SF6Sensor[0]=0x30;
                        SF6Sensor[1]=Uart1DataBuffer[0];
                        SF6Sensor[2]=Uart1DataBuffer[1];
                        SF6Sensor[3]=Uart1DataBuffer[2];
                        SF6Sensor[4]=Uart1DataBuffer[3];
                        break;
                    }
                    else if(Uart1DataBufferLength==10)//长度正确，保存数据
                    {
                        SF6Sensor[0]=Uart1DataBuffer[0];
                        SF6Sensor[1]=Uart1DataBuffer[1];
                        SF6Sensor[2]=Uart1DataBuffer[2];
                        SF6Sensor[3]=Uart1DataBuffer[3];
                        SF6Sensor[4]=Uart1DataBuffer[4];
                        break;
                    }
                    else 
                    {
                        break;
                    }
                    
                }
            }

            //判断数据头,如果trueshujuUart2为true，则不接受'$'，从而不识别数据头，从而不接受数据
            if (Uart1RXBuffer[0] == 0x20 )
            {
                if (Uart1spaceFlag == 1)
                {
                    Uart1FrameHeadFlag = 1;
                }
                else
                {
                    Uart1spaceFlag = 1;
                }
            }
            else
            {
                Uart1spaceFlag = 0;
            }
            break ;
        case 4:break                                                     ;  // Vector 4 - TXIFG
        default: break                                                   ;  
    }  
}

/*******************************************************************************
串口2数据接收模块 噪声传感器
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
              if(Uart2RXBuffer[0]==0x01)//地址位
              {
                Uart2DataBuffer[0]=Uart2RXBuffer[0];
                Uart2RXFlag=0x01;
              }
              Uart2InstructFlag=0x00;//接收失败
              break;
            case 0x01:
              if(Uart2RXBuffer[0]==0x04)//功能码
              {
                Uart2DataBuffer[1]=Uart2RXBuffer[0];
                Uart2RXFlag=0x02;
                break;
              }
              Uart2InstructFlag=0x00;//接收失败
              Uart2RXFlag=0x00;
              break;
            case 0x02:
              if(Uart2RXBuffer[0]==0x02)//数据量
              {
                Uart2DataBuffer[2]=Uart2RXBuffer[0];
                Uart2RXFlag=0x03;
                break;
              }
              Uart2InstructFlag=0x00;//接收失败
              Uart2RXFlag=0x00;
              break;
            case 0x03://数据位高字节
              Uart2DataBuffer[3]=Uart2RXBuffer[0];
              volumeSensor[0]=Uart2RXBuffer[0];
              Uart2RXFlag=0x04;
              break;
            case 0x04://数据位低字节
              Uart2DataBuffer[4]=Uart2RXBuffer[0];
              volumeSensor[1]=Uart2RXBuffer[0];
              Uart2RXFlag=0x05;
              
              break;
            case 0x05://校验位低字节
              Uart2DataBuffer[5]=Uart2RXBuffer[0];
              Uart2RXFlag=0x06;
              break;
            case 0x06://校验位高字节
              Uart2DataBuffer[6]=Uart2RXBuffer[0];
              Uart2RXFlag=0x00;
              unsigned int crc=Uart2DataBuffer[6]*256+Uart2DataBuffer[5];
              if(crc==CRC16RTU(Uart2DataBuffer,5))
              {
                Uart2InstructFlag=0x01;//接收成功
              }
              else
              {
                Uart2InstructFlag=0x00;//接收失败
              }
              
              break;
            }
            break ;
        case 4:break ;  // Vector 4 - TXIFG
        default: break ;  
    }
}
/*******************************************************************************
串口3数据接收模块 485总线
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
            switch (Uart3RXFlag)
            {
            case 0x00://数据头
              if(Uart3RXBuffer[0]==0xFE)
              {
                  Uart3RXFlag=0x01;
              }
              break;
            case 0x01://数据长度位
              Uart3DataBfLength=Uart3RXBuffer[0];//记录数据长度
              Uart3DataNum=0;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存数据长度
              Uart3RXFlag=0x02;//找到数据长度位
              break;
            case 0x02://目的地址
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存目的地址
              Uart3InstructAim=Uart3RXBuffer[0];
              Uart3RXFlag=0x03;//找到数据长度位
              break;
            case 0x03://源地址
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存源地址
              Uart3InstructSource=Uart3RXBuffer[0];
              Uart3RXFlag=0x04;//找到数据长度位
              break;
            case 0x04://指令编号
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//保存指令编号
              Uart3InstructNum=Uart3RXBuffer[0];
              Uart3RXFlag=0x05;//找到数据长度位
              break;
            case 0x05://数据位和CRC位
              Uart3DataNum++;//用于计数
              Uart3DataBuffer[Uart3DataNum]=Uart3RXBuffer[0];//数据位
              if(Uart3DataBfLength<=Uart3DataNum)//接收预计数量的数据
              {
                unsigned char crc=Uart3RXBuffer[0];//保存CRC位
                if((Uart3InstructAim==0x02) &&(crc == CRC(Uart3DataBuffer,Uart3DataBfLength)))//目的地址正确，校验正确
                {
                  Uart3InstructFlag=0x01;
                  Uart3InstructNum=Uart3ExtrInstr(Uart3DataBuffer ,Uart3DataBfLength);
                  if(Uart3InstructNum==0x18)//程序更新指令
                  {
                      if(Uart3UpdateDataNum==0x00)//第一帧
                      {
                          updateState=0x01;//软件更新状态
                          dataLost=0x00;//
                          UpdateDataNum=Uart3UpdateDataNum;
                          addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart3UpdateDataNum*249),Uart3DataBuffer+6 ,(unsigned char)(Uart3DataBfLength-6));
                      }
                      if(Uart3UpdateDataNum!=0x00)//不是第一帧
                      {
                          if(UpdateDataNum+1==Uart3UpdateDataNum)//连续帧
                          {
                              UpdateDataNum=Uart3UpdateDataNum;
                              addrEeprom=eeprom_write_page(0x0000+(unsigned int)(Uart3UpdateDataNum*249),Uart3DataBuffer+6 ,(unsigned char)(Uart3DataBfLength-6));
                          }
                          else if(UpdateDataNum==Uart3UpdateDataNum)//重复帧
                          {}
                          else
                          {
                            dataLost=0x01;//数据丢失
                          }
                      }  
                      __delay_cycles(8000);
                      unsigned char buffer_1[5]={0x05 ,0x01 ,0x02, 0x18, Uart3UpdateDataNum};
                      unsigned char crc=CRC(buffer_1,5);
                      unsigned char buffer[7]={0xFE , 0x05 ,0x01,0x02, 0x18 , Uart3UpdateDataNum , crc};
                      Uart3_send(buffer,7);
                      
                      if(Uart3UpdateStatus==0x01)//最后一帧
                      {
                          if(dataLost==0x00)
                          {
                              delay_ms(5);
                              unsigned char c=0x55;
                              eeprom_writepage(0xFFFF,&c,1);//写标志，告诉引导代码有现成的程序，可以直接更新                              
                              
                              UCA0IE &= ~UCRXIE ; // 关闭接收中断
                              UCA1IE &= ~UCRXIE ; // 关闭接收中断
                              UCA2IE &= ~UCRXIE ; // 关闭接收中断
                              UCA3IE &= ~UCRXIE ; // 关闭接收中断
                              TA0CCTL0 &= ~CCIE ; // 关闭时钟中断
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
                  Uart3InstructFlag=0x00;
                }
                Uart3RXFlag=0x00;
              }
              break;
            default :
              Uart3RXFlag=0x00;
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

unsigned char Uart3ExtrInstr(unsigned char *buffer ,int length)
{
    switch (buffer[3])
    {
        case 0x01://握手请求指令
            return 0x01;
            
        case 0x02://自检请求指令
            return 0x02;
                    
        case 0x03://心跳指令
            return 0x03;

        case 0x04://复位请求指令
            return 0x04;

        case 0x05://机器人版本信息请求指令
            return 0x05;

        case 0x06://机器人充电状态请求指令
            return 0x06;

        case 0x07://机器人运动总里程请求指令
            return 0x07;

        case 0x08://机器人状态请求指令
            return 0x08;
            
        case 0x09://电量信息请求指令
            return 0x09;
            
        case 0x18://软件更新
          //Uart3UpdateID=buffer[2];//设备ID
          Uart3UpdateDataNum=buffer[4];//数据序号
          Uart3UpdateStatus=buffer[5];//状态位
          return 0x18;

        default:
            return 0x00;//表示提取数据失败
    }
}





