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

extern unsigned char Uart1InstructFlag;//接收到指令标志
extern unsigned char distData[2];
extern unsigned char isObstacle;

extern unsigned char moveDir;//0x00表示停止，0x01表示前，0x02表示后，0x03表示下
extern long servo_X_PWM;//X舵机的PWM给定值
extern long servo_Y_PWM;//Y舵机的PWM给定值
extern unsigned char Uart2moveRes[3];//主控板移动指令
extern unsigned char Uart2movetoRes[2];//主控板移动到指令
extern unsigned char Uart2ptzRollRes[3];//主控板移动云台指令
extern unsigned char Uart2ptzRollToRes[4];//主控板云台移动到指令
unsigned char yuntaiRollUp=0x00;
unsigned char yuntaiRollDown=0x00;
unsigned char yuntaiRollLeft=0x00;
unsigned char yuntaiRollRight=0x00;

extern unsigned char timer0Flag;//定时时间到标志
extern unsigned int timer0Count;//定时时间到标志
unsigned char motor_status_Z=0;//编码器状态
extern unsigned char encoder_status_Z;//编码器状态
extern unsigned char Uart2InstructFlag;//接收到指令标志
extern unsigned char Uart2InstructNum;//指令编号
extern int count_BM;//编码器计数

extern unsigned char movetoZ_status=0x00;
extern unsigned char movetoZ_flag=0x00;//电动推杆移动到指令标志
int Iweizhi=0;

unsigned char detectNum=0x00;

unsigned char isObstacleFront=0x00;//前侧障碍物
unsigned char isObstacleFront1=0x00;//前侧障碍物
unsigned char isObstacleFront2=0x00;//前侧障碍物
unsigned char isObstacleFront3=0x00;//前侧障碍物
unsigned char isObstacleFront4=0x00;//前侧障碍物
char isObstacleFrontCount=0x00;//前方障碍物计数
unsigned char isObstacleBelow=0x00;//下面障碍物
/*******************************************************************************
获取数据采集板数据、状态信息
*******************************************************************************/
void DetectState()
{
    unsigned char buffer_1[15];
    buffer_1[0]=0x0F;//长度位
    buffer_1[1]=0x01;//目的地址位
    buffer_1[2]=0x03;//源地址位
    buffer_1[3]=0x01;//指令编号
    
    Iweizhi=count_BM/10;//编码器读数和实际位置的关系
    buffer_1[4]=(unsigned char)(Iweizhi>>8);
    buffer_1[5]=(unsigned char)Iweizhi;
    
    buffer_1[6]=motor_status_Z;//编码器状态
    buffer_1[7]=encoder_status_Z;//编码器状态
    buffer_1[8]=movetoZ_status;//Z移动到位状态
    
    long servo_X_Angle=(((long)2000*(7500-servo_X_PWM))/2000)+9000;
    long servo_Y_Angle=(((long)2000*(servo_Y_PWM-12000))/2000)+9000;
    
    unsigned char CyuntaiX[2];
    unsigned char CyuntaiY[2];
    CyuntaiX[0]=(unsigned char)((servo_X_Angle>>8) & 0XFF);
    CyuntaiX[1]=(unsigned char)(servo_X_Angle & 0XFF);
    
    CyuntaiY[0]=(unsigned char)((servo_Y_Angle>>8) & 0XFF);
    CyuntaiY[1]=(unsigned char)(servo_Y_Angle & 0XFF);
    
    buffer_1[9]=CyuntaiX[0];
    buffer_1[10]=CyuntaiX[1];
    
    buffer_1[11]=CyuntaiY[0];
    buffer_1[12]=CyuntaiY[1];
    buffer_1[13]=0x00;//云台状态
    buffer_1[14]= isObstacleBelow | (isObstacleFront<<2);//障碍物状态
    unsigned char crc = CRC(buffer_1,15);
    
    unsigned char buffer[17];
    buffer[0]=0xfe;
    for(int i=0;i<15;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[16]=crc;
    //delay_ms(1);
    Uart2_send(buffer,17);
  
}
/*******************************************************************************
移动电动推杆，根据指令中包含的数据解析出运动方向和运动速度
然后运动，返回接收到回复
*******************************************************************************/
void Move_Z()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x02 ,0x07};
    Uart2_send(buffer,6); //先回复主控，表示接收到指令了
    
    char uDir=Uart2moveRes[0];
    //int uSpe=Uart2moveRes[2]+Uart2moveRes[1]*256;
    int delta=1000;//转速变量跟uSpe有关，是个正比关系
    if ((uDir==0x00)&&(((P2IN&LS1_LA)!=LS1_LA)&&((P1IN&LS2_LA)!=LS2_LA)))//电机正传，伸出去
    {
        P1OUT |= LA_IN1 ;//低
        P1OUT &= ~LA_IN2 ;//高
        
        TA0CCR2=delta;
    }
    else if((uDir==0x01)&&((P1IN&LS3_LA)!=LS3_LA))//缩回来，利用限位开关限制，防止伸缩杆在限位0点时仍能向里缩
    {
        P1OUT |= LA_IN2 ;//低
        P1OUT &= ~LA_IN1 ;//高
        
        TA0CCR2=delta;
    }

    else{}
    
}
/*******************************************************************************
停止移动电动推杆
*******************************************************************************/
void Stop_Z()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x03 ,0x36};
    Uart2_send(buffer,6); //先回复主控，表示接收到指令了
    
    P1OUT |= LA_IN1 ;//高
    P1OUT |= LA_IN2 ;//高
        
    TA0CCR2=0;
    
}
/*******************************************************************************
电动推杆运动到指定位置指令
*******************************************************************************/
void MoveToZ(void)
{
    Iweizhi=count_BM/10;//编码器读数和实际位置的关系
  //位置指令执行：      到达指定位置的速度规划及到达判断
  if(movetoZ_flag == 1)                     
  {

    if(encoder_status_Z == 1)                                                        //3，编码器故障导致任务失败   ；
    {
      movetoZ_flag = 0;
      movetoZ_status = 3;                                                //3，编码器故障导致任务失败   ；
      //电机停止
      P1OUT |= LA_IN1 ;//高
      P1OUT |= LA_IN2 ;//高
      TA0CCR2=0;    
    }
    
    else                                                                        //编码器正常工作   ；正常按照位置规划速度
    {
      //分段加减速//方向判断
      int weizhiTo=(int)(Uart2movetoRes[0])*256+(int)(Uart2movetoRes[1]);
//        int Iweizhi = Ibianmaqi/10;                          //?????
      if(weizhiTo - Iweizhi>0)
      {/*
        if(((P2IN&LS1_LA)==LS1_LA)||((P1IN&LS2_LA)==LS2_LA))//局放伸缩杆头部限位到
        {
          movetoZ_flag = 0;
          movetoZ_status = 1;                                                //1，认为移动到位   ；
          //电机停止
          P1OUT |= LA_IN1 ;//高
          P1OUT |= LA_IN2 ;//高
          TA0CCR2=0;    
        }*/
//          UpDownFlag = 1;                                //表示需要下行
        if(weizhiTo - Iweizhi < 2)                     //5为到达点的提前余量，待定
        {
                                                       //**********停止下行
          P1OUT |= LA_IN1 ;//高
          P1OUT |= LA_IN2 ;//高
          TA0CCR2=0;
          movetoZ_status = 1;                           //位置指令状态：0,正常默认状态  ；1，到达位置   ；2，被其他运动控制指令打断   ；3，编码器故障导致任务失败   ；
          movetoZ_flag = 0;                           //不再进行位置移动指令的执行            
        }
        else //if(weizhiTo - Iweizhi >= 2 && weizhiTo - Iweizhi <= 20)                     //待定
        {
                                                       //距离与速度设定值（占空比）关系
            P1OUT |= LA_IN1 ;//低
            P1OUT &= ~LA_IN2 ;//高
        
            TA0CCR2=1000;
        
        }/*
        else if(weizhiTo - Iweizhi > 20 && weizhiTo - Iweizhi <= 50)                     //待定
        {
                                                       //速度设定值关系
            P1OUT |= LA_IN1 ;//低
            P1OUT &= ~LA_IN2 ;//高
            TA0CCR2=1000;
        
        }
        else
        {
                                                       //下行的最大速度设定值   
            P1OUT |= LA_IN1 ;//低
            P1OUT &= ~LA_IN2 ;//高
            TA0CCR2=2000;
        }
        */
      }
      else if(weizhiTo - Iweizhi <= 0)
      {
//          UpDownFlag = 0;                                //0表示需要上行 ；1表示需要下行
        if(weizhiTo - Iweizhi > -2)                     //5为到达点的提前余量，待定
        {
                                                       //**********停止上行
            P1OUT |= LA_IN1 ;//低
            P1OUT |= LA_IN2 ;//高
            TA0CCR2=0;
            movetoZ_status = 1;                           //到达位置            
            movetoZ_flag = 0; 
        }
        else //if(weizhiTo - Iweizhi <= -2 && weizhiTo - Iweizhi >= -20)                     //待定
        {
                                                       //距离与速度设定值（占空比）关系
            P1OUT &= ~LA_IN1 ;//低
            P1OUT |= LA_IN2 ;//高
            TA0CCR2=1000;
        
        }/*
        else if(weizhiTo - Iweizhi < -20 && weizhiTo - Iweizhi >= -50)                     //待定
        {
                                                       //速度设定值关系
            P1OUT &= ~LA_IN1 ;//低
            P1OUT |= LA_IN2 ;//高
            TA0CCR2=1000;
        
        }
        else
        {
                                                       //下行的最大速度设定值 
            P1OUT &= ~LA_IN1 ;//低
            P1OUT |= LA_IN2 ;//高
            TA0CCR2=2000;
        }   
 */       
      }          
    }
                                            
  }
}

/*******************************************************************************
电动推杆移动到指定位置 清除变量指令
*******************************************************************************/
void MoveToZ_END(void)
{
    P1OUT |= LA_IN1 ;//高
    P1OUT |= LA_IN2 ;//高
    TA0CCR2=0;
    //
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x05 ,0x90};
    Uart2_send(buffer,6); //先回复主控，表示接收到指令了
}
/*******************************************************************************
移动云台指令
*******************************************************************************/
void InfraPtzRoll ()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x06 ,0xc3};
    Uart2_send(buffer,6); //先回复主控，表示接收到指令了
    
    if(Uart2ptzRollRes[0]==0)//上
    {
        yuntaiRollUp=1;
        yuntaiRollDown=0;
        yuntaiRollLeft=0;
        yuntaiRollRight=0;
    }
    else if(Uart2ptzRollRes[0]==1)//下
    {
        yuntaiRollUp=0;
        yuntaiRollDown=1;
        yuntaiRollLeft=0;
        yuntaiRollRight=0;
    }
    else if(Uart2ptzRollRes[0]==2)//左
    {
        yuntaiRollUp=0;
        yuntaiRollDown=0;
        yuntaiRollLeft=1;
        yuntaiRollRight=0;
    }
    else if(Uart2ptzRollRes[0]==3)//右
    {
        yuntaiRollUp=0;
        yuntaiRollDown=0;
        yuntaiRollLeft=0;
        yuntaiRollRight=1;
    }
    else
    {
        yuntaiRollUp=0;
        yuntaiRollDown=0;
        yuntaiRollLeft=0;
        yuntaiRollRight=0;
    }
}
void RollYunTai()
{
    //转动云台
        
    if(yuntaiRollUp==1&&servo_Y_PWM<14000)
    {
        servo_Y_PWM=servo_Y_PWM+10;
        TA1CCR1=servo_Y_PWM;
    }
    if(yuntaiRollDown==1&&servo_Y_PWM>10000)
    {
        servo_Y_PWM=servo_Y_PWM-10;
        TA1CCR1=servo_Y_PWM;
    }
    if(yuntaiRollLeft==1&&servo_X_PWM<9500)
    {
        servo_X_PWM=servo_X_PWM+10;
        TA1CCR2=servo_X_PWM;
    }
    if(yuntaiRollRight==1&&servo_X_PWM>5500)
    {
        servo_X_PWM=servo_X_PWM-10;
        TA1CCR2=servo_X_PWM;
    }
        
}
/*******************************************************************************
停止移动云台指令
*******************************************************************************/
void InfraPtzStop ()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x07 ,0xF2};
    Uart2_send(buffer,6); //先回复主控，表示接收到指令了
    
    yuntaiRollUp=0;
    yuntaiRollDown=0;
    yuntaiRollLeft=0;
    yuntaiRollRight=0;
}
/*******************************************************************************
移动云台到指定位置
*******************************************************************************/
void InfraPtzRollToReq ()
{
    long servo_X_Angle=(long)(Uart2ptzRollToRes[0]*256+Uart2ptzRollToRes[1]);
    long servo_Y_Angle=(long)(Uart2ptzRollToRes[2]*256+Uart2ptzRollToRes[3]);
    
    servo_X_PWM=(long)(7500-2000*(servo_X_Angle-9000)/2000);
    servo_Y_PWM=(long)(2000*(servo_Y_Angle-9000)/2000+12000);
    if(servo_X_PWM<5500)
      servo_X_PWM=5500;
    if(servo_X_PWM>9500)
      servo_X_PWM=9500;
    if(servo_Y_PWM<10000)
      servo_Y_PWM=10000;
    if(servo_Y_PWM>14000)
      servo_Y_PWM=14000;
    
    TA1CCR1=servo_Y_PWM ;
    TA1CCR2=servo_X_PWM ;
    
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x08 ,0xDC};
    Uart2_send(buffer,6); //先回复主控，表示接收到指令了
}
/*******************************************************************************
最小时间片
*******************************************************************************/
void MinTimeslice()//20ms时间片
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
指令处理函数
*******************************************************************************/
void InstructProcess(void)
{
    if(Uart2InstructFlag==0x01)//有主控指令
    {
        Uart2InstructFlag=0x00;//清标志
        switch (Uart2InstructNum)
        {
        case 0x01://状态查询指令
          DetectState();
          break;
          
        case 0x02://移动电动推杆指令
          Move_Z();
          break;
          
        case 0x03://停止移动电动推杆指令
          Stop_Z();
          break;
     
        case 0x04://移动电动推杆到指定位置指令
          {
              unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x04 ,0xa1};
              Uart2_send(buffer,6); //先回复主控，表示接收到指令了
              
              movetoZ_status=0;
              movetoZ_flag = 1;
          }
          break;
          
        case 0x05://移动到指定位置完成，清除变量指令
          {
              movetoZ_flag = 0;
              movetoZ_status = 0;                        //位置指令结束 强制恢复
              //同时电机停止  
              MoveToZ_END();
          }
          break;
        case 0x06://移动云台
          InfraPtzRoll ();
          break;
          
        case 0x07://停止移动云台
          InfraPtzStop ();
          break;
        case 0x08:
          InfraPtzRollToReq ();
          break;
          
        case 0x18:
        {
            
            
        }
        default :
          break;

        }
    }
}
/*******************************************************************************
超声波传感器读取，返回有无
*******************************************************************************/
unsigned char DetectObstacle(void)
{
    if(Uart2InstructFlag==0x00)//没有主控指令
    {
        unsigned char a=0x01;
        Uart1_send(&a,1);//串口发送数据请求指令
        Init_Timer0_B7();//开10ms定时器
        while(1)
        {
            if(timer0Flag==1)
            {
                //Uart1InstructFlag = 0;
                //isObstacle=0x00;
                //return 0x00;
              break;
            }
        }
        if(Uart1InstructFlag == 1)//传感器有数据传回来
        {
            if(isObstacle==0x01)//存在障碍物
            {
                Uart1InstructFlag = 0;
                isObstacle=0x00;
                return 0x01;
            }
            else
            {
                Uart1InstructFlag = 0;
                isObstacle=0x00;
                return 0x00;
            }
        }
    }
    return 0x00;
}
/*******************************************************************************
按照时序检测5个超声波避障传感器，超声波传感器检测时间是100ms
*******************************************************************************/
void DetectSensor(void)
{
    if(Uart2InstructFlag != 0x01)//没有主控指令
    {
        MinTimeslice();//20ms时间片，纯粹延时20ms。
        detectNum++;
        RollYunTai();//缓慢转动云台
        switch(detectNum)
        {
          case 1:
          {
            unsigned int dist=distData[0]*256+distData[1];
            if((dist>250)&&(dist<600))
            {
              isObstacleFront4=0x01;//有障碍物
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront4=0x00;//无障碍物
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                P8OUT |= USC1;//选择第二个超声波传感器 前左下  有故障 头有问题
                P8OUT &= ~USC2;
                P8OUT &= ~USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//串口发送数据请求指令
            }
            break;
          }
          case 2:break;
          case 3:break;
          case 4:break;
          case 5:break;
          
          case 6:
          {
            
            unsigned int dist=distData[0]*256+distData[1];
            if((dist>250)&&(dist<600))
            {
              isObstacleFront1=0x01;//有障碍物
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront1=0x00;//无障碍物
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x03)
            {
                
                P8OUT &= ~USC1;//选择第一个超声波传感器 前底面
                P8OUT &= ~USC2;
                P8OUT &= ~USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//串口发送数据请求指令
                
            }
            break;
          }
          case 7:break;
          case 8:break;
          case 9:break;
          case 10:break;
          
          case 11:
          {
            unsigned int dist=distData[0]*256+distData[1];
            if((dist>250)&&(dist<400))
            {
              isObstacleBelow=0x01;//有障碍物
            }
            else
              isObstacleBelow=0x00;//无障碍物
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                
                P8OUT &= ~USC1;//选择第三个超声波传感器 前右下 有误检
                P8OUT &= ~USC2;
                P8OUT |= USC3;
                
                unsigned char a=0x01;
                Uart1_send(&a,1);//串口发送数据请求指令
                
            }
            break;
          }
          case 12:break;
          case 13:break;
          case 14:break;
          case 15:break;
          
          case 16:
          {
            unsigned int dist=distData[0]*256+distData[1];
            if((dist>250)&&(dist<600))
            {
              isObstacleFront2=0x01;//有障碍物
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront2=0x00;//无障碍物
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                
                P8OUT &= ~USC1;//选择第四个超声波传感器 D 前左上
                P8OUT |= USC2;
                P8OUT |= USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//串口发送数据请求指令
                
            }
            break;
          }
          case 17:break;
          case 18:break;
          case 19:break;
          case 20:break;
          
          case 21:
          {
            unsigned int dist=distData[0]*256+distData[1];
            if((dist>250)&&(dist<600))
            {
              isObstacleFront3=0x01;//有障碍物
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront3=0x00;//无障碍物
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                
                P8OUT |= USC1;//选择第五个超声波传感器 前右上
                P8OUT &= ~USC2;
                P8OUT |= USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//串口发送数据请求指令
                
            }
            break;
          }
          case 22:break;
          case 23:break;
          case 24:break;
          case 25:break;
          
          default:
          {
            detectNum=0;
            
            if((isObstacleFront1==0)&&(isObstacleFront2==0)&&(isObstacleFront3==0)&&(isObstacleFront4==0))
            {
              isObstacleFront=0;
            }
            /*
            if(isObstacleFrontCount>=2)
            {
              isObstacleFront=1;
              isObstacleFrontCount=0;
            }
            else
            {
              isObstacleFront=0;
              isObstacleFrontCount=0;
            }
            */
            break;
          }
        }
    }
}