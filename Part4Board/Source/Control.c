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
#include "eeprom.h"
int timerCount=0;
int detectNum=0x00;
int writeCount=0;

extern unsigned char coordFail;//坐标校准故障
unsigned char AM_status=0x00;//温湿度传感器状态，0x00表示正常，0x01表示故障
extern unsigned char motorX_status=0x00;//小车电机状态
int motorX_FailCount=0x00;//x电机故障次数
extern unsigned char motorY_status=0x00;//伸缩杆电机状态
int motorY_FailCount=0x00;//x电机故障次数
extern unsigned char EncoderX_status=0x00;//小车编码器状态，0x00表示正常，0x01表示故障
extern unsigned char EncoderY_status=0x00;//伸缩杆编码器状态，0x00表示正常，0x01表示故障
extern unsigned char MoveToRES_X=0x00;//
extern unsigned char MoveToRES_Y=0x00;//
extern long LlocationX;
extern long LlocationY;
long LtotalmileEncoder=0;
extern unsigned long Ltotalmile=0;
unsigned char x_moving=0;
unsigned char y_moving=0;
extern unsigned char moveDir=0x00;

int humidity=0;
int tempreature=0;
unsigned char Chumidity[2]={0x00};
unsigned char Ctempreature[2]={0x00};
unsigned char encoderXFaultNum=0;//编码器读取错误次数
unsigned char encoderYFaultNum=0;//编码器读取错误次数

extern unsigned char obstacle_status;//障碍物情况
extern unsigned char Uart1moveRes[4];//move指令相关信息
extern unsigned char Uart1movetoResX[4];//moveto指令X相关信息
extern unsigned char Uart1movetoResY[2];//moveto指令Y相关信息

unsigned char movetoX_status=0x00;
unsigned char movetoX_flag=0x00;//小车移动到指令标志
unsigned char movetoY_status=0x00;
unsigned char movetoY_flag=0x00;//电动推杆移动到指令标志
/*******************************************************************************
获取数据采集板数据、状态信息
*******************************************************************************/
void DetectState()
{
    unsigned char buffer_1[31];
    buffer_1[0]=0x1f;//长度位
    buffer_1[1]=0x01;//目的地址位
    buffer_1[2]=0x04;//源地址位
    buffer_1[3]=0x01;//指令编号
    
    unsigned char CweizhiX[4];
    CweizhiX[3]=(unsigned char)(LlocationX);
    CweizhiX[2]=(unsigned char)(LlocationX>>8);
    CweizhiX[1]=(unsigned char)(LlocationX>>16);
    CweizhiX[0]=(unsigned char)(LlocationX>>24);
    
    buffer_1[4]=CweizhiX[0];
    buffer_1[5]=CweizhiX[1];
    buffer_1[6]=CweizhiX[2];
    buffer_1[7]=CweizhiX[3];
    buffer_1[8]=motorX_status;//x电机状态
    buffer_1[9]=EncoderX_status;//X编码器状态
    buffer_1[10]=movetoX_status;//x位置指令状态
    
    unsigned char CweizhiY[4];
    CweizhiY[1]=(unsigned char)(LlocationY);
    CweizhiY[0]=(unsigned char)(LlocationY>>8);
    
    buffer_1[11]=CweizhiY[0];
    buffer_1[12]=CweizhiY[1];
    buffer_1[13]=motorY_status;//y电机状态
    buffer_1[14]=EncoderY_status;//y编码器状态
    buffer_1[15]=movetoY_status;//y位置指令状态
    
    unsigned char CTem[2];
    unsigned char CHum[2];
    
    CTem[1]=(unsigned char)(tempreature);
    CTem[0]=(unsigned char)(tempreature>>8);
    CHum[1]=(unsigned char)(humidity);
    CHum[0]=(unsigned char)(humidity>>8);
    
    buffer_1[16]=CHum[0];
    buffer_1[17]=CHum[1];
    buffer_1[18]=CTem[0];
    buffer_1[19]=CTem[1];
    
    buffer_1[20]=AM_status;//温湿度传感器状态
    
    /*
    总里程10B
    */
    unsigned char Ctotalmile[10];
    Ctotalmile[9]=(unsigned char)(Ltotalmile/1%10)+48;
    Ctotalmile[8]=(unsigned char)(Ltotalmile/10%10)+48;
    Ctotalmile[7]=(unsigned char)(Ltotalmile/100%10)+48;
    Ctotalmile[6]=(unsigned char)(Ltotalmile/1000%10)+48;
    Ctotalmile[5]=(unsigned char)(Ltotalmile/10000%10)+48;
    Ctotalmile[4]=(unsigned char)(Ltotalmile/100000%10)+48;
    Ctotalmile[3]=(unsigned char)(Ltotalmile/1000000%10)+48;
    Ctotalmile[2]=(unsigned char)(Ltotalmile/10000000%10)+48;
    Ctotalmile[1]=(unsigned char)(Ltotalmile/100000000%10)+48;
    Ctotalmile[0]=(unsigned char)(Ltotalmile/1000000000%10)+48;
    
    buffer_1[21]=Ctotalmile[0];
    buffer_1[22]=Ctotalmile[1];
    buffer_1[23]=Ctotalmile[2];
    buffer_1[24]=Ctotalmile[3];
    buffer_1[25]=Ctotalmile[4];
    buffer_1[26]=Ctotalmile[5];
    buffer_1[27]=Ctotalmile[6];
    buffer_1[28]=Ctotalmile[7];
    buffer_1[29]=Ctotalmile[8];
    buffer_1[30]=Ctotalmile[9];
    unsigned char crc = CRC(buffer_1,31);
    
    unsigned char buffer[33];
    buffer[0]=0xfe;
    for(int i=0;i<31;i++)
    {
        buffer[i+1]=buffer_1[i];
    }
    buffer[32]=crc;
    //char buffer[13]={0x24 ,0x24 ,0x01 ,0x04 ,0x30 ,0x32 ,0x04 ,0x72 ,0x21};
    Uart1_send(buffer,33);
}
/*******************************************************************************
移动指令
*******************************************************************************/
void Move()
{
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x02,0xa9};
    Uart1_send(buffer,6);
    
    if(Uart1moveRes[0]==0x00)//移动小车
    {
        x_moving=0x01;//小车移动中。
        int uSpe=Uart1moveRes[3]+Uart1moveRes[2]*256;
        int delta=150*uSpe;//转速变量跟uSpe有关，是个正比关系
        if(delta>1500)
          delta=1500;
        if (Uart1moveRes[1]==0x00)//电机正传，增大前进
        {
            moveDir=0x01;
            //if((obstacle_status&BIT2)!=BIT2)
            {
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+delta;
            }
        }
        else if(Uart1moveRes[1]==0x01)//减小后退
        {
            moveDir=0x02;
            //if((obstacle_status&BIT1)!=BIT1)
            {
                P1OUT &= ~DCMSTOP1 ;//电机运行  
                TA0CCR4=4000-delta;
            }
            
        }
        else{}
    }
    else if(Uart1moveRes[0]==0x01)//移动伸缩杆
    {
        y_moving=0x01;//伸缩杆移动中。
        int uSpe=Uart1moveRes[3]+Uart1moveRes[2]*256;
        int delta=500*uSpe;//转速变量跟uSpe有关，是个正比关系
        if(delta>2000)
          delta=2000;
        if (Uart1moveRes[1]==0x01)//电机正传，减小上升
        {
            moveDir=0x04;
            //if((obstacle_status&BIT0)!=BIT0)
            {
                P1OUT &= ~DCMSTOP2 ;//电机运行
                TA0CCR1=4000-delta;
            }
        }
        else if(Uart1moveRes[1]==0x00)//增大下降
        {
            moveDir=0x03;
            P1OUT &= ~DCMSTOP2 ;//电机运行  
            TA0CCR1=4000+delta;
        }
        else{}
    }
    else
    {
      
    }
}
/*******************************************************************************
停止指令
*******************************************************************************/
void Stop()
{
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x03,0x98};
    Uart1_send(buffer,6);
    x_moving=0x00;//小车不在移动中。
    y_moving=0x00;//伸缩杆不在移动中。
    
    moveDir=0x00;
    P1OUT |= DCMSTOP1 ;//小车电机停止 
    TA0CCR4=4000;
    P1OUT |= DCMSTOP2 ;//电机停止
    TA0CCR1=4000;
}
/*******************************************************************************
位置指令执行函数，速度规划及到达判断
*******************************************************************************/
void MoveToX(void)
{
  obstacle_status=0x00;
  //位置指令执行：      到达指定位置的速度规划及到达判断
  if(movetoX_flag == 1)                     
  {

    if((EncoderX_status == 1)||(motorX_status==1))    //3，编码器或电机故障导致任务失败   ；
    {
      movetoX_flag = 0;
      movetoX_status = 3;                                                //3，编码器故障导致任务失败   ；
      //电机停止
      P1OUT |= DCMSTOP1 ;//小车电机停止 
      TA0CCR4=4000;     
      moveDir=0x00;
    }
    else                                                                        //编码器正常工作   ；正常按照位置规划速度
    {
      //分段加减速//方向判断
      long locationXTo=(long)(Uart1movetoResX[0])*256*256*256
                    +(long)(Uart1movetoResX[1])*256*256
                    +(long)(Uart1movetoResX[2])*256
                    +(long)(Uart1movetoResX[3]);
//        int Iweizhi = Ibianmaqi/10;                          //?????
      
      if(locationXTo - LlocationX>0)
      {
          moveDir=0x01;
          if((obstacle_status&0x04)!=0x04)//前方无障碍
          {
              if(locationXTo - LlocationX < 2)                     //5为到达点的提前余量，待定
              {
                                                             //**********停止下行
                P1OUT |= DCMSTOP1 ;//小车电机停止 
                TA0CCR4=4000;     
                moveDir=0x00;
                movetoX_status = 1;                           //位置指令状态：0,正常默认状态  ；1，到达位置   ；
                                                              //2，被其他运动控制指令打断   ；3，编码器故障导致任务失败   ；
                movetoX_flag = 0;                           //不再进行位置移动指令的执行            
              }
              else if(locationXTo - LlocationX >= 2 && locationXTo - LlocationX <= 20)                     //待定
              {
                                                             //距离与速度设定值（占空比）关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+100;
              
              }
              else if(locationXTo - LlocationX > 20 && locationXTo - LlocationX <= 200)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+300;
              
              }
              else if(locationXTo - LlocationX > 100 && locationXTo - LlocationX <= 200)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+600;
              
              }
              else if(locationXTo - LlocationX > 200 && locationXTo - LlocationX <= 300)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+900;
              
              }
              else if(locationXTo - LlocationX > 300 && locationXTo - LlocationX <= 400)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+1200;
              
              }
              else if(locationXTo - LlocationX > 400 && locationXTo - LlocationX <= 1000)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+1500;
              
              }
              else if(locationXTo - LlocationX > 1000 && locationXTo - LlocationX <= 1200)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+1800;
              
              }
              else if(locationXTo - LlocationX > 1200 && locationXTo - LlocationX <= 1400)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+2100;
              
              }
              else if(locationXTo - LlocationX > 1400 && locationXTo - LlocationX <= 1600)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+2400;
              
              }
              else if(locationXTo - LlocationX > 1600 && locationXTo - LlocationX <= 1800)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+2700;
              
              }
              else if(locationXTo - LlocationX > 1800 && locationXTo - LlocationX <= 2000)                     //待定
              {
                                                             //速度设定值关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+3000;
              
              }
              
              else
              {
                                                             //下行的最大速度设定值   
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000+3200;
              }
          }
//          UpDownFlag = 1;                                //表示需要下行
          else 
          {
              movetoX_flag = 0;
              movetoX_status = 5;                                                //5，障碍物存在导致任务失败   ；
              //电机停止
              P1OUT |= DCMSTOP1 ;//电机运行
                TA0CCR4=4000;  
                moveDir=0x00;
          }
        
      }
      else if( locationXTo - LlocationX <= 0 )
      {
          moveDir=0x02;
          if(((obstacle_status&0x02)!=0x02)||(LlocationX<1000))//后方无障碍
          {
                    //          UpDownFlag = 0;                                //0表示需要上行 ；1表示需要下行
              if(locationXTo - LlocationX > -2)                     //5为到达点的提前余量，待定
              {
                                                             //**********停止上行
                  P1OUT |= DCMSTOP1 ;//电机运行
                TA0CCR4=4000;
                moveDir=0x00;
                  movetoX_status = 1;                           //到达位置   
                  movetoX_flag = 0;                           //不再进行位置移动指令的执行             
              }
              else if(locationXTo - LlocationX <= -2 && locationXTo - LlocationX >= -20)                     //待定
              {
                                                             //距离与速度设定值（占空比）关系
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-100;
              
              }
              else if(locationXTo - LlocationX < -20 && locationXTo - LlocationX >= -100)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-300;
              
              }
              else if(locationXTo - LlocationX < -100 && locationXTo - LlocationX >= -200)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-600;
              
              }
              else if(locationXTo - LlocationX < -200 && locationXTo - LlocationX >= -300)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-900;
              
              }
              else if(locationXTo - LlocationX < -300 && locationXTo - LlocationX >= -400)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-1200;
              
              }
              else if(locationXTo - LlocationX < -400 && locationXTo - LlocationX >= -1000)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-1500;
              
              }
              else if(locationXTo - LlocationX < -1000 && locationXTo - LlocationX >= -1200)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-1800;
              
              }
              else if(locationXTo - LlocationX < -1200 && locationXTo - LlocationX >= -1400)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-2100;
              
              }
              else if(locationXTo - LlocationX < -1400 && locationXTo - LlocationX >= -1600)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-2400;
              
              }
              else if(locationXTo - LlocationX < -1600 && locationXTo - LlocationX >= -1800)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-2700;
              
              }
              else if(locationXTo - LlocationX < -1800 && locationXTo - LlocationX >= -2000)                     //待定
              {
                                                             //速度设定值关系
                P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-3000;
              
              }
              else
              {
                                                             //下行的最大速度设定值 
                  P1OUT &= ~DCMSTOP1 ;//电机运行
                TA0CCR4=4000-3200;
              }          
          }
          else 
          {
              
              movetoX_flag = 0;
              movetoX_status = 5;                                                //5，障碍物存在导致任务失败   ；
              //电机停止
              P1OUT |= DCMSTOP1 ;//电机运行
              TA0CCR4=4000;
              moveDir=0x00;
          }

      }
      
    }
                                            
  }
}
/*******************************************************************************
位置指令   完成  清除变量指令     小车的函数
*******************************************************************************/
void MoveToX_END(void)
{
    movetoX_flag = 0;
    movetoX_status = 0;                        //位置指令结束 强制恢复
    
    moveDir=0x00;
    
    P1OUT |= DCMSTOP1 ;//电机运行
    TA0CCR4=4000;
    
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x05,0x3e};
    Uart1_send(buffer,6);
}
/*******************************************************************************
位置指令执行函数，速度规划及到达判断
*******************************************************************************/
void MoveToY(void)
{
  //位置指令执行：      到达指定位置的速度规划及到达判断
  if(movetoY_flag == 1)                     
  {
    if((EncoderY_status == 1)||(motorY_status==1)) //3，电机或编码器故障导致任务失败   ；
    {
      movetoY_flag = 0;
      movetoY_status = 3;                                                //3，编码器故障导致任务失败   ；
      //电机停止
      P1OUT |= DCMSTOP2 ;//电机运行
      TA0CCR1=4000;
    }
    else                                                                        //编码器正常工作   ；正常按照位置规划速度
    {
      //分段加减速//方向判断
      long locationYTo=(long)(Uart1movetoResY[0])*256
                    +(long)(Uart1movetoResY[1]);
//        int Iweizhi = Ibianmaqi/10;                          //?????
      if(locationYTo - LlocationY>0) //向下运动
      {
        moveDir=0x03;
        if((obstacle_status&0x01)!=0x01)//下方无障碍物
        {
//          UpDownFlag = 1;                                //表示需要下行
          if(locationYTo - LlocationY <= 1)                     //5为到达点的提前余量，待定
          {
                                                         //**********停止下行
            P1OUT |= DCMSTOP2 ;//电机运行
            TA0CCR1=4000;
            movetoY_status = 1;                           //位置指令状态：0,正常默认状态  ；1，到达位置   ；2，被其他运动控制指令打断   ；3，编码器故障导致任务失败   ；
            movetoY_flag = 0;                           //不再进行位置移动指令的执行            
          }
          else if(locationYTo - LlocationY > 1 && locationYTo - LlocationY <= 50)                     //待定
          {
                                                         //距离与速度设定值（占空比）关系
              P1OUT &= ~DCMSTOP2 ;//电机运行
              TA0CCR1=4000+100;
          
          }
          else if(locationYTo - LlocationY > 50 && locationYTo - LlocationY <= 200)                     //待定
          {
                                                         //速度设定值关系
              P1OUT &= ~DCMSTOP2 ;//电机运行
                  TA0CCR1=4000+300;
          
          }
          else
          {
                                                         //下行的最大速度设定值   
              P1OUT &= ~DCMSTOP2 ;//电机运行
                  TA0CCR1=4000+800;
          }
        }
        else
        {
            movetoY_flag = 0;
            movetoY_status = 4;                                                //4，障碍物存在导致任务失败   ；
            //电机停止
            P1OUT |= DCMSTOP2 ;//电机运行
            TA0CCR1=4000;
        }
      }
      else if(locationYTo - LlocationY <= 0)
      {
        moveDir=0x04;
//          UpDownFlag = 0;                                //0表示需要上行 ；1表示需要下行
        if(locationYTo - LlocationY >= -1)                     //5为到达点的提前余量，待定
        {
                                                       //**********停止上行
            P1OUT |= DCMSTOP2 ;//电机运行
            TA0CCR1=4000;
            movetoY_status = 1;                           //到达位置            
        }
        else if(locationYTo - LlocationY < -1 && locationYTo - LlocationY >= -50)                     //待定
        {
                                                       //距离与速度设定值（占空比）关系
            P1OUT &= ~DCMSTOP2 ;//电机运行
                TA0CCR1=4000-100;
        
        }
        else if(locationYTo - LlocationY < -50 && locationYTo - LlocationY >= -200)                     //待定
        {
                                                       //速度设定值关系
          P1OUT &= ~DCMSTOP2 ;//电机运行
                TA0CCR1=4000-300;
        
        }
        else
        {
                                                       //下行的最大速度设定值 
            P1OUT &= ~DCMSTOP2 ;//电机运行
                TA0CCR1=4000-800;
        }          
      }
      else {}
      
    }
                                            
  }
}
/*******************************************************************************
位置指令   完成  清除变量指令
*******************************************************************************/
void MoveToY_END(void)
{
    movetoY_flag = 0;
    movetoY_status = 0;                        //位置指令结束 强制恢复
    moveDir=0x00;
    
    P1OUT |= DCMSTOP2 ;//电机运行
    TA0CCR1=4000;
    
    unsigned char buffer[6]={0xfe,0x04 ,0x01 ,0x04 ,0x07,0x5c};
    Uart1_send(buffer,6);
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
          Move();
          break;
        case 0x03:
          Stop();
          break;
        case 0x04:
          {
              movetoX_flag = 1;
              movetoX_status=0;
              unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x04 ,0x04 ,0x0f};
              Uart1_send(buffer,6); //先回复主控，表示接收到指令了
              motorX_status=0x00;//电机无故障
              break;
          }
        case 0x05:
          MoveToX_END();
          break;
        case 0x06:
          {
              movetoY_flag = 1;
              movetoY_status=0;
              unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x04 ,0x06 ,0x6d};
              Uart1_send(buffer,6); //先回复主控，表示接收到指令了
              motorY_status=0x00;//电机无故障
              break;
          }
        case 0x07:
          MoveToY_END();
          break;
          
        case 0x08:
        {
          unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x04 ,0x08 ,0x72};
          Uart1_send(buffer,6); //先回复主控，表示接收到指令了
          Reset();
          break;
        }
        case 0x18:
        {
            
            
        }
        default :
          break;
          
          
        }
    }
}
/*******************************************************************************
读取小车绝对式编码器值，根据这个值正比计算出X轴位置
*******************************************************************************/
void DetectEncoder1()
{
    if(Uart1InstructFlag != 0x01)
    {
        ReadEncoder1();//读取小车编码器
        timerCount++;
    }
}
/*******************************************************************************
读取伸缩杆绝对式编码器值，根据这个值正比计算出Y轴位置
*******************************************************************************/
void DetectEncoder2()
{
    if(Uart1InstructFlag != 0x01)
    {
      ReadEncoder2();//读取伸缩杆编码器
      timerCount++;
    }
}
/*******************************************************************************
温湿度参数检测，保存总里程数据，FC00-FC03，备份FC04-FC08
*******************************************************************************/
void DetectTAH()
{
    if((Uart1InstructFlag != 0x01) && (timerCount>=100))
    {
      DetectAM();//读取温湿度
      MinTimeslice();//延时
      //writeCount++;
      
      if((x_moving==0x01)||(movetoX_flag == 1))//小车移动的时候才写
      {
          unsigned char Ctotalmile[4]={0x00};
          
          Ctotalmile[3]=(unsigned char)Ltotalmile;
          Ctotalmile[2]=(unsigned char)(Ltotalmile>>8);
          Ctotalmile[1]=(unsigned char)(Ltotalmile>>16);
          Ctotalmile[0]=(unsigned char)(Ltotalmile>>24);
          eeprom_writepage(0xFC00 , Ctotalmile, 4);//写入eeprom里面
          delay_ms(5);
          //eeprom_writepage(0xFC04 , Ctotalmile, 4);//写入eeprom里面
          //delay_ms(5);
      }
      
      timerCount=0;
    }
}
/*******************************************************************************
读取小车绝对式编码器值，根据这个值正比计算出X轴位置
*******************************************************************************/

void ReadEncoder1()
{
    P5DIR |= ENCODER_SET ;
    P5OUT &= ~ENCODER_SET ;
    
    long encoderX=LencoderX;
    EncoderX_status=0x01;
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
    Uart2_send(bf,4);//先发送指令
    Init_Timer0_B7();//开10ms定时器
    while (1)
    {
        if(Uart2InstructFlag == 1)//编码器回复
        {
            encoderXFaultNum=0;//编码器出错次数
            EncoderX_status=0x00;//接收成功
            Uart2InstructFlag = 0;//清标志
            LencoderXOld=encoderX;//记录两次编码器读数，方便计算总里程
        }
        else
        {
        }
        if(timer0Flag==1)
        {
            encoderXFaultNum++;//编码器出错次数加一
            break;
        }
    }
    if(encoderXFaultNum>5)
    {
        encoderXFaultNum=0;//编码器出错次数清零
        EncoderX_status=0x01;//接受失败
    }
    
    long i=LencoderX-LencoderXOld;
    if(i>0)
    {
        if(i<1000)
        {
          LtotalmileEncoder+=i;
        }
        else{}
    }
    if(i<0)
    {
        if(i>-1000)
        {
          LtotalmileEncoder-=i;
        }
        else{}
    }
    if(movetoX_flag==1)//x轴运动到
    {
        if(i==0)//编码器值不变，认为电机有问题
        {
            if(motorX_status==0x00)
                motorX_FailCount++;//x电机故障次数
        }
        else
        {
            motorX_FailCount=0;
            motorX_status=0x00;//电机无故障
        }
    }
    if(motorX_FailCount>100)//故障次数
    {
        motorX_status=0x01;//电机故障
    }
    
    Ltotalmile+=LtotalmileEncoder/280;
    LtotalmileEncoder=LtotalmileEncoder%280;
  
}
/*******************************************************************************
读取伸缩杆绝对式编码器值，根据这个值正比计算出Y轴位置
*******************************************************************************/

void ReadEncoder2()
{
    P5DIR |= ENCODER_SET ;
    P5OUT &= ~ENCODER_SET ;
    
    long encoderY=LencoderY;
    EncoderY_status=0x01;
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
    Uart3_send(bf,4);//先发送指令
    Init_Timer0_B7();//开10ms定时器
    while (1)
    {
        if(Uart3InstructFlag == 1)//编码器回复
        {
            encoderYFaultNum=0;//编码器出错次数
            EncoderY_status=0x00;//接收成功
            Uart3InstructFlag = 0;//清标志
            LencoderYOld=encoderY;//记录两次编码器读数
        }
        else
        {
        }
        if(timer0Flag==1)
        {
            encoderYFaultNum++;//编码器出错次数加一
            break;
        }
    }
    if(encoderYFaultNum>5)
    {
        encoderYFaultNum=0;//编码器出错次数清零
        EncoderY_status=0x01;//接受失败
    }
    
    long i=LencoderY-LencoderYOld;
    
    if(movetoY_flag==1)//y轴运动到
    {
        if(i==0)//编码器值不变，认为电机有问题
        {
            if(motorY_status==0x00)
                motorY_FailCount++;//x电机故障次数
        }
        else
        {
            motorY_FailCount=0;
            motorY_status=0x00;//电机无故障
        }
    }
    if(motorY_FailCount>200)//故障次数
    {
        motorY_status=0x01;//电机故障
        
    }
  
}

/*******************************************************************************
温湿度参数检测请求
*******************************************************************************/
/////////////////////////////////////////////
#define DQ BIT7
#define DQ_DIR  P4DIR
#define DQ_IN   P4IN
#define DQ_OUT  P4OUT
/*-----------------------------------------*/
#define DQ1 DQ_OUT |= DQ
#define DQ0 DQ_OUT &=~DQ
#define DIR_IN DQ_DIR &=~DQ
#define DIR_OUT DQ_DIR |=DQ

unsigned char Read_AM2301(void)
{
    int count=0;
    unsigned char rdata = 0;
    for(int i=0;i<8;i++)
    {
        rdata <<= 1;
        count=0;
        while((DQ_IN & DQ)!=DQ)
        {
            count++;
            if(count>1000)
            {
              break;
            }
        }
        delay_us(33);
        if((DQ_IN & DQ)==DQ)  rdata|=BIT0;
        else 
         rdata &= ~BIT0;
        count=0;
        while((DQ_IN & DQ)==DQ)
        {
            count++;
            if(count>1000)
            {
              break;
            }
        }     
    } 
    return  rdata;
}

void DetectAM()
{
    int count=0;
    //温湿度传感器工作时，绝对不允许被打断
    _DINT() ;//禁止中断
    unsigned char a,b,c,d,f;
    unsigned int e;
    DIR_OUT;     //io口输出
    DQ0;             //io口置0  
    delay_us(1000);    //
    DIR_IN;         //io口输入
    delay_us(20);
    
    count=0;
    while((DQ_IN&DQ)==DQ)
    {
        count++;
        if(count>1000)
        {
          break;
        }
    }
    
    delay_us(60);
    
    count=0;
    while(!(DQ_IN&DQ))
    {
        count++;
        if(count>1000)
        {
          break;
        }
    }
    
    delay_us(60);
    while((DQ_IN&DQ)==DQ)
    {
        count++;
        if(count>1000)
        {
          break;
        }
    }
    
    delay_us(50);
    a= Read_AM2301();
    b= Read_AM2301();
    c= Read_AM2301();
    d= Read_AM2301();
    f= Read_AM2301();
    e=(unsigned int) (a+b+c+d);
    //P3OUT &= ~SI1;
    if(e>255) e=e-255;
    if(e=f)
    {
        AM_status=0;//温湿度传感器未故障
        humidity=(a*256+b);
        tempreature=(c*256+d);
        
        Chumidity[0]=a;
        Chumidity[1]=b;
        Ctempreature[0]=c;
        Ctempreature[1]=d;
        
    }
    else //获取失败
    {
        AM_status=1;//温湿度传感器故障
        humidity=0;
        tempreature=0;
    }
    _EINT();         //开总中断5
}

/*******************************************************************************
时间片轮循检测传感器值，编码器，温湿度传感器
*******************************************************************************/
void DetectSensor(void)
{
    detectNum++;
    if(detectNum>=100)//温湿度两秒测一次
    {
      DetectTAH(); 
      detectNum=0;
    }
    switch(detectNum%2)
    {
        case 0:
          DetectEncoder1();
          break;
        case 1:
          DetectEncoder2();
          break;
        default:
          break;
    }
}
/*******************************************************************************
初始化编码器
*******************************************************************************/
void Reset()
{
  //小车编码器初始化
  {
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    
    //配置编码器方向,44 30 30 43 07 0d
    unsigned char buffer1[6]={0x44,0x30,0x30,0x43,0x07,0x0d};
    Uart2_send(buffer1,6);
    
    delay_ms(200);
    
    
    //配置编码器goggnzuomoshi,44 30 30 43 07 0d
    unsigned char buffer2[6]={0x44,0x30,0x30,0x4e,0x10,0x0d};
    Uart2_send(buffer2,6);
    
    delay_ms(200);
    
    //配置编码器最大值
    unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x50,0x4c,0x30 ,0x30 ,0x31 ,0x30 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
    Uart2_send(buffer,16);
        
    delay_ms(200);
    
    //配置编码器初始值
    while(1)
    {
        unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x4a ,0x30 ,0x30 ,0x30 ,0x31 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
        Uart2_send(buffer,15);
        delay_ms(100);
        if(Uart2InstructFlag == 1)
        {
            Uart2InstructFlag = 0;
            break;
        }
        
    }
    
    P5OUT  &= ~ENCODER_SET;
    
    while(1)
    {
        unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart2_send(bf,4);//先发送指令
        delay_ms(20);
        if(Uart2InstructFlag == 1)
        {
            Uart2InstructFlag = 0;
            break;
        }
        
    }
    
    long int bmq_1=0;
    if(LencoderX<=1000000)
    {
        bmq_1=2000000-LencoderX;
    }
    else 
    {
        bmq_1=12000000-LencoderX;
    }
    unsigned char bmq_2[10]={0x00};
    bmq_2[9]=(unsigned char)(bmq_1%10+48);
    bmq_2[8]=(unsigned char)(bmq_1/10%10+48);
    bmq_2[7]=(unsigned char)(bmq_1/100%10+48);
    bmq_2[6]=(unsigned char)(bmq_1/1000%10+48);
    bmq_2[5]=(unsigned char)(bmq_1/10000%10+48);
    bmq_2[4]=(unsigned char)(bmq_1/100000%10+48);
    bmq_2[3]=(unsigned char)(bmq_1/1000000%10+48);
    bmq_2[2]=(unsigned char)(bmq_1/10000000%10+48);
    bmq_2[1]=(unsigned char)(bmq_1/100000000%10+48);
    bmq_2[0]=(unsigned char)(bmq_1/1000000000%10+48);
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    delay_ms(100);
    
    while(1)
    {
        unsigned char buffer1[16]={0x44,0x30 ,0x30 ,0x4a ,bmq_2[0] ,bmq_2[1] ,bmq_2[2] ,bmq_2[3] ,bmq_2[4] ,bmq_2[5],bmq_2[6],bmq_2[7],bmq_2[8],bmq_2[9],0x0d};
        Uart2_send(buffer1,15);
        delay_ms(20);
        if(Uart2InstructFlag == 1)
        {
           Uart2InstructFlag = 0;
            break;
        }
        
    }
    delay_ms(100);
    P5OUT  &= ~ENCODER_SET;
    
    delay_ms(200);
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart2_send(bf,4);//先发送指令
        delay_ms(20);
  }
  
  
  //伸缩杆编码器初始化
  {
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    delay_ms(200);
    //配置编码器方向,44 30 30 43 07 0d
    unsigned char buffer1[6]={0x44,0x30,0x30,0x43,0x06,0x0d};
    Uart3_send(buffer1,6);
    
    delay_ms(200);
    
    
    //配置编码器gongzuomoshi,44 30 30 43 07 0d
    unsigned char buffer2[6]={0x44,0x30,0x30,0x4e,0x10,0x0d};
    Uart3_send(buffer2,6);
    
    delay_ms(200);
    
    
    //unsigned char buffer1[6]={0x44,0x30 ,0x30 ,0x4E,0x10,0x0d};
    //Uart3_send(buffer1,6);
    //配置编码器最大值
    unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x50,0x4c,0x30 ,0x30 ,0x31 ,0x30 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
    Uart3_send(buffer,16);
        
    delay_ms(200);
    //配置编码器初始值
    while(1)
    {
        unsigned char buffer[16]={0x44,0x30 ,0x30 ,0x4a ,0x30 ,0x30 ,0x30 ,0x31 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
        Uart3_send(buffer,15);
        delay_ms(100);
        if(Uart3InstructFlag == 1)
        {
            Uart3InstructFlag = 0;
            break;
        }
        
    }
    
    P5OUT  &= ~ENCODER_SET;
    
    while(1)
    {
        unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart3_send(bf,4);//先发送指令
        delay_ms(20);
        if(Uart3InstructFlag == 1)
        {
            Uart3InstructFlag = 0;
            break;
        }
        
    }
    
    long int bmq_1=0;
    if(LencoderY<=1000000)
    {
        bmq_1=2000000-LencoderY;
    }
    else 
    {
        bmq_1=12000000-LencoderY;
    }
    unsigned char bmq_2[10]={0x00};
    bmq_2[9]=(unsigned char)(bmq_1%10+48);
    bmq_2[8]=(unsigned char)(bmq_1/10%10+48);
    bmq_2[7]=(unsigned char)(bmq_1/100%10+48);
    bmq_2[6]=(unsigned char)(bmq_1/1000%10+48);
    bmq_2[5]=(unsigned char)(bmq_1/10000%10+48);
    bmq_2[4]=(unsigned char)(bmq_1/100000%10+48);
    bmq_2[3]=(unsigned char)(bmq_1/1000000%10+48);
    bmq_2[2]=(unsigned char)(bmq_1/10000000%10+48);
    bmq_2[1]=(unsigned char)(bmq_1/100000000%10+48);
    bmq_2[0]=(unsigned char)(bmq_1/1000000000%10+48);
    P5DIR |= ENCODER_SET ;
    P5OUT  |= ENCODER_SET;
    delay_ms(100);
    
    while(1)
    {
        unsigned char buffer1[16]={0x44,0x30 ,0x30 ,0x4a ,bmq_2[0] ,bmq_2[1] ,bmq_2[2] ,bmq_2[3] ,bmq_2[4] ,bmq_2[5],bmq_2[6],bmq_2[7],bmq_2[8],bmq_2[9],0x0d};
        Uart3_send(buffer1,15);
        delay_ms(20);
        if(Uart3InstructFlag == 1)
        {
           Uart3InstructFlag = 0;
            break;
        }
        
    }
    delay_ms(100);
    P5OUT  &= ~ENCODER_SET;
    
    delay_ms(200);
    unsigned char bf[4] = {0x44,0x30,0x30,0x0d};
        Uart3_send(bf,4);//先发送指令
        delay_ms(20);
    
  }
  
    /*
    //读取编码器配置
    P5DIR |= BME ;
    P5OUT  |= BME;
    delay_ms(2000);
    unsigned char buffer[]={0x44,0x00 ,0x41 ,0x0d};
    Uart0_send(buffer,4);
    delay_ms(2000);
    */
    /*
    //配置最大值
    P5DIR |= BME ;
    P5OUT  |= BME;
    delay_ms(2000);
    unsigned char buffer[]={0x44,0x30 ,0x30 ,0x50,0x4c,0x30 ,0x30 ,0x31 ,0x30 ,0x30 ,0x30,0x30,0x30,0x30,0x30,0x0d};
    Uart0_send(buffer,16);
    delay_ms(2000);
    */
    
    /*
    //配置方向
    P5DIR |= BME ;
    P5OUT  |= BME;
    delay_ms(2000);
    unsigned  char buffer1[]={0x44,0x30 ,0x30 ,0x43 ,0x07 ,0x0d };
    Uart0_send(buffer1,6);
    delay_ms(2000);
    */
}