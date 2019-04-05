//===========================================================================//
//                                                                           //
// 文件：  Control.c                                                         //
// 说明：  功能函数，根据通信协议做相应动作                                  //
// 编译：  IAR Embedded Workbench IDE for msp430 v5.50                       //
// 版本：  v1.0                                                              //
// 时间：  2017/03/16                                                        //
// 编写：  LaSeine                                                           //
// 版权：  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "Control.h"
#include "SysInit.h"
#include "Uart.h"

extern unsigned char chrgstateNow=0x00;
extern unsigned char chrgstateOld=0x00;
int chrgFailCount=0;//充电故障次数
unsigned char battery=0x00;
extern float voltage=0.0;
extern float current=0.0;
extern unsigned char Uart1InstructFlag;//接收到指令标志
extern unsigned char Uart1InstructNum;//指令编号
int timerCount=0;

/*******************************************************************************
获取数据采集板数据、状态信息
*******************************************************************************/
void DetectState()
{
    if((chrgstateNow==0x01)&&(0x00==chrgstateOld))
    {
      P4OUT |= RELAY1;
    }
    else if((chrgstateNow==0x00)&&(0x01==chrgstateOld))
    {
      P4OUT &= ~RELAY1;
    }
    else
    {}
    unsigned char buffer_1[8];
    buffer_1[0]=0x08;//长度位
    buffer_1[1]=0x01;//目的地址位
    buffer_1[2]=0x05;//源地址位
    buffer_1[3]=0x01;//指令编号
    if(chrgstateNow==0x01)//在充电状态的情况下，判断充电是不是有故障
    {
        if((current-1.0<1.0)&&(voltage<27.5))//电压小于一定值的情况下，电流差小于一定值，认为没有充上电
        {
            chrgFailCount++;
            //
        }
        else 
            chrgFailCount=0;
            //
    }
    if(chrgFailCount>=50)
    {
      chrgFailCount=50;
      buffer_1[4]=0x01;
    }
    else 
      buffer_1[4]=0x00;
    
    
    if(voltage>28.6)
      battery=100;
    else if(voltage>28.0)
      battery=95;
    else if(voltage>27.6)
      battery=90;
    else if(voltage>27.2)
      battery=85;
    else if(voltage>26.8)
      battery=80;
    else if(voltage>26.5)
      battery=75;
    else if(voltage>26.0)
      battery=70;
    else if(voltage>25.8)
      battery=65;
    else if(voltage>25.4)
      battery=60;
    else if(voltage>25.2)
      battery=55;
    else if(voltage>24.9)
      battery=50;
    else if(voltage>24.7)
      battery=45;
    else if(voltage>24.6)
      battery=40;
    else if(voltage>24.4)
      battery=35;
    else if(voltage>24.3)
      battery=30;
    else if(voltage>24.1)
      battery=25;
    else if(voltage>24.)
      battery=20;
    else if(voltage>23.7)
      battery=15;
    else if(voltage>23.5)
      battery=10;
    else
      battery=5;
    
    buffer_1[5]=battery;
    
    int Icurrent = (int)(current*100);
    buffer_1[6]=(unsigned char)(Icurrent>>8);
    buffer_1[7]=(unsigned char)(Icurrent);
    
    //buffer_1[5]=100;
    unsigned char crc = CRC(buffer_1,8);
    
    unsigned char buffer[10];
    buffer[0]=0xfe;
    for(int i=0;i<8;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[9]=crc;
    Uart1_send(buffer,10);
  
}
/*******************************************************************************
指令处理函数
*******************************************************************************/
void InstructProcess()
{
    if(Uart1InstructFlag==0x01)//有主控指令
    {
        Uart1InstructFlag=0x00;//清标志
        switch (Uart1InstructNum)
        {
        case 0x01:
          DetectState();
          break;
          
        case 0x02:
          
          break;
        
     
        default :
          break;
          
          
        }
    }
}
/*******************************************************************************
检测电压电流
*******************************************************************************/
void DetectOwnState()
{
    if(Uart1InstructFlag != 0x01)
    {
        DectctCur();//测电流
        timerCount++;
        if(timerCount==1)
        {
            DetectVol();//测电压
        }
        if(timerCount>3000)//一分钟
        {
            timerCount=0;
        }
    }
}
/*******************************************************************************
最小时间片
*******************************************************************************/
void MinTimeslice()
{
    Init_Timer0_B7();
    while(1)
    {
        if(timer0Flag==1)
        {
            timer0Count++;
            break;
        }
    }
}
/*******************************************************************************
检测电压时间片
*******************************************************************************/
void DetectVol()
{       //测电压
    Init_Timer0_B7();
    voltage=detectVoltage();
    while(1)
    {
        if(timer0Flag==1)
        {
            break;
        }
        
    }
}
/*******************************************************************************
检测电流时间片
*******************************************************************************/
void DectctCur()
{
    //测电流
    Init_Timer0_B7();
    current=detectCurrent();
    while(1)
    {
        if(timer0Flag==1)
        {
            break;
        }
        
    }
}
/*******************************************************************************
电压检测，分压检测
*******************************************************************************/
float detectVoltage()
{
 
    P6SEL |= BIT5; // 选择P6.5作为模拟信号的输入端

    ADC12CTL0 = ADC12ON; // 使能转换模块

    REFCTL0|=REFMSTR+REFVSEL_2+REFON;

    //使能REF管理，内部参考电压选择2.5v、打开内部参考电压

    ADC12CTL1 = ADC12SHP;// // 选择脉冲触发模式、单通道单次次转换模式

    ADC12MCTL0= ADC12SREF_1+ADC12INCH_5; //选择参考电压源、现在a5通道

    ADC12CTL0 |= ADC12ENC; // 使能转换模块

    for ( unsigned char i=0; i<0x30; i++);

    ADC12CTL0 |= ADC12SC; // 开始转换

    while ((ADC12CTL1 & ADC12BUSY));

    float voltage= (((float)ADC12MEM0/4096)*2.5)*24.6605-9.4441; //观察Average
  
  /*
    // 只有在ADC12ENC复位的情况下才可以操作  
    // ADC12SHT1X ADC12SHT0X ADC12MSC ADC12REF2_5V ADC12REFON ADC12ON  
    ADC12CTL0 &= ~ADC12ENC;  
  
    // 设置采样保持时间，最大时间周期以提高转换精度  
    // 注意MSP430F5438没有REF模块，片内基准无效  
    // 操作ADC12REF2_5V ，ADC12REFON并无意义  
    //ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON;  
//    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON +  
//                ADC12REF2_5V + ADC12REFON;  
    // 采样保持脉冲来自采样定时器  
    REFCTL0|=REFMSTR+REFVSEL_2+REFON;
    ADC12CTL1 = ADC12SHP;  
    // 关闭内部内部温度检测以降低功耗，注意或操作否则修改转换精度  
    ADC12CTL2 |= ADC12TCOFF ;  
    // 基准电压选择AVCC，并选择1通道——(AVCC-AVSS)/2  
    ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_5;  
  
    __delay_cycles(75);  
    // ADC12使能  
    ADC12CTL0 |= ADC12ENC; 
    
    
    ADC12CTL0 |= ADC12SC;                   // 启动转换  
    while ( !(ADC12IFG & BIT0) );           // 等待转换完成  
  
            // 被转换的通道为通道11 (AVCC-AVSS)/2;  
            // 此时转换的精度为12位——4096  
            // AVCC通过一个电感和LDO的输出端连接  
            // 打印LDO输出电压，保留3位精度  
    float voltage = (ADC12MEM0  / 4096.0) * 33 * 2.4/2.5; 
     */
    return voltage;
   
}

/*******************************************************************************
电流检测
*******************************************************************************/
float detectCurrent()
{
    P6SEL |= BIT6; // 选择P6.5作为模拟信号的输入端

    ADC12CTL0 = ADC12ON; // 使能转换模块

    REFCTL0|=REFMSTR+REFVSEL_2+REFON;

    //使能REF管理，内部参考电压选择2.5v、打开内部参考电压

    ADC12CTL1 = ADC12SHP;// // 选择脉冲触发模式、单通道单次次转换模式

    ADC12MCTL0= ADC12SREF_1+ADC12INCH_6; //选择参考电压源、现在a5通道

    ADC12CTL0 |= ADC12ENC; // 使能转换模块

    for ( unsigned char i=0; i<0x30; i++);

    ADC12CTL0 |= ADC12SC; // 开始转换

    while ((ADC12CTL1 & ADC12BUSY));
    float electric = ( 2.5-(ADC12MEM0  / 4096.0 * 2.5) )*10;
  /*
    // 只有在ADC12ENC复位的情况下才可以操作  
    // ADC12SHT1X ADC12SHT0X ADC12MSC ADC12REF2_5V ADC12REFON ADC12ON  
    ADC12CTL0 &= ~ADC12ENC;  
  
    // 设置采样保持时间，最大时间周期以提高转换精度  
    // 注意MSP430F5438没有REF模块，片内基准无效  
    // 操作ADC12REF2_5V ，ADC12REFON并无意义  
    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON;  
//    ADC12CTL0 = ADC12SHT0_15 + ADC12SHT1_15 + ADC12ON +  
//                ADC12REF2_5V + ADC12REFON;  
    // 采样保持脉冲来自采样定时器  
    ADC12CTL1 = ADC12SHP;  
    // 关闭内部内部温度检测以降低功耗，注意或操作否则修改转换精度  
    ADC12CTL2 |= ADC12TCOFF ;  
    // 基准电压选择AVCC，并选择1通道——(AVCC-AVSS)/2  
    ADC12MCTL0 = ADC12SREF_0 + ADC12INCH_6;  
  
    __delay_cycles(75);  
    // ADC12使能  
    ADC12CTL0 |= ADC12ENC; 
    
    
    ADC12CTL0 |= ADC12SC;                   // 启动转换  
    while ( !(ADC12IFG & BIT0) );           // 等待转换完成  
  
            // 被转换的通道为通道11 (AVCC-AVSS)/2;  
            // 此时转换的精度为12位——4096  
            // AVCC通过一个电感和LDO的输出端连接  
            // 打印LDO输出电压，保留3位精度  
    float electric = ADC12MEM0  / 4096.0 * 3.3; 
  */
    return electric;
    
}



