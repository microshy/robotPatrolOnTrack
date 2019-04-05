//===========================================================================//
//                                                                           //
// �ļ���  Control.c                                                         //
// ˵����  ���ܺ���������ͨ��Э������Ӧ����                                  //
// ���룺  IAR Embedded Workbench IDE for msp430 v5.50                       //
// �汾��  v1.0                                                              //
// ʱ�䣺  2017/03/16                                                        //
// ��д��  LaSeine                                                           //
// ��Ȩ��  NJUST                                                             //
//                                                                           //
//===========================================================================//
#include "msp430f5438a.h"
#include "PIN_DEF.h"
#include "Control.h"
#include "SysInit.h"
#include "Uart.h"

extern unsigned char Uart1InstructFlag;//���յ�ָ���־
extern unsigned char distData[2];
extern unsigned char isObstacle;

extern unsigned char moveDir;//0x00��ʾֹͣ��0x01��ʾǰ��0x02��ʾ��0x03��ʾ��
extern long servo_X_PWM;//X�����PWM����ֵ
extern long servo_Y_PWM;//Y�����PWM����ֵ
extern unsigned char Uart2moveRes[3];//���ذ��ƶ�ָ��
extern unsigned char Uart2movetoRes[2];//���ذ��ƶ���ָ��
extern unsigned char Uart2ptzRollRes[3];//���ذ��ƶ���ָ̨��
extern unsigned char Uart2ptzRollToRes[4];//���ذ���̨�ƶ���ָ��
unsigned char yuntaiRollUp=0x00;
unsigned char yuntaiRollDown=0x00;
unsigned char yuntaiRollLeft=0x00;
unsigned char yuntaiRollRight=0x00;

extern unsigned char timer0Flag;//��ʱʱ�䵽��־
extern unsigned int timer0Count;//��ʱʱ�䵽��־
unsigned char motor_status_Z=0;//������״̬
extern unsigned char encoder_status_Z;//������״̬
extern unsigned char Uart2InstructFlag;//���յ�ָ���־
extern unsigned char Uart2InstructNum;//ָ����
extern int count_BM;//����������

extern unsigned char movetoZ_status=0x00;
extern unsigned char movetoZ_flag=0x00;//�綯�Ƹ��ƶ���ָ���־
int Iweizhi=0;

unsigned char detectNum=0x00;

unsigned char isObstacleFront=0x00;//ǰ���ϰ���
unsigned char isObstacleFront1=0x00;//ǰ���ϰ���
unsigned char isObstacleFront2=0x00;//ǰ���ϰ���
unsigned char isObstacleFront3=0x00;//ǰ���ϰ���
unsigned char isObstacleFront4=0x00;//ǰ���ϰ���
char isObstacleFrontCount=0x00;//ǰ���ϰ������
unsigned char isObstacleBelow=0x00;//�����ϰ���
/*******************************************************************************
��ȡ���ݲɼ������ݡ�״̬��Ϣ
*******************************************************************************/
void DetectState()
{
    unsigned char buffer_1[15];
    buffer_1[0]=0x0F;//����λ
    buffer_1[1]=0x01;//Ŀ�ĵ�ַλ
    buffer_1[2]=0x03;//Դ��ַλ
    buffer_1[3]=0x01;//ָ����
    
    Iweizhi=count_BM/10;//������������ʵ��λ�õĹ�ϵ
    buffer_1[4]=(unsigned char)(Iweizhi>>8);
    buffer_1[5]=(unsigned char)Iweizhi;
    
    buffer_1[6]=motor_status_Z;//������״̬
    buffer_1[7]=encoder_status_Z;//������״̬
    buffer_1[8]=movetoZ_status;//Z�ƶ���λ״̬
    
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
    buffer_1[13]=0x00;//��̨״̬
    buffer_1[14]= isObstacleBelow | (isObstacleFront<<2);//�ϰ���״̬
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
�ƶ��綯�Ƹˣ�����ָ���а��������ݽ������˶�������˶��ٶ�
Ȼ���˶������ؽ��յ��ظ�
*******************************************************************************/
void Move_Z()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x02 ,0x07};
    Uart2_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
    
    char uDir=Uart2moveRes[0];
    //int uSpe=Uart2moveRes[2]+Uart2moveRes[1]*256;
    int delta=1000;//ת�ٱ�����uSpe�йأ��Ǹ����ȹ�ϵ
    if ((uDir==0x00)&&(((P2IN&LS1_LA)!=LS1_LA)&&((P1IN&LS2_LA)!=LS2_LA)))//������������ȥ
    {
        P1OUT |= LA_IN1 ;//��
        P1OUT &= ~LA_IN2 ;//��
        
        TA0CCR2=delta;
    }
    else if((uDir==0x01)&&((P1IN&LS3_LA)!=LS3_LA))//��������������λ�������ƣ���ֹ����������λ0��ʱ����������
    {
        P1OUT |= LA_IN2 ;//��
        P1OUT &= ~LA_IN1 ;//��
        
        TA0CCR2=delta;
    }

    else{}
    
}
/*******************************************************************************
ֹͣ�ƶ��綯�Ƹ�
*******************************************************************************/
void Stop_Z()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x03 ,0x36};
    Uart2_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
    
    P1OUT |= LA_IN1 ;//��
    P1OUT |= LA_IN2 ;//��
        
    TA0CCR2=0;
    
}
/*******************************************************************************
�綯�Ƹ��˶���ָ��λ��ָ��
*******************************************************************************/
void MoveToZ(void)
{
    Iweizhi=count_BM/10;//������������ʵ��λ�õĹ�ϵ
  //λ��ָ��ִ�У�      ����ָ��λ�õ��ٶȹ滮�������ж�
  if(movetoZ_flag == 1)                     
  {

    if(encoder_status_Z == 1)                                                        //3�����������ϵ�������ʧ��   ��
    {
      movetoZ_flag = 0;
      movetoZ_status = 3;                                                //3�����������ϵ�������ʧ��   ��
      //���ֹͣ
      P1OUT |= LA_IN1 ;//��
      P1OUT |= LA_IN2 ;//��
      TA0CCR2=0;    
    }
    
    else                                                                        //��������������   ����������λ�ù滮�ٶ�
    {
      //�ֶμӼ���//�����ж�
      int weizhiTo=(int)(Uart2movetoRes[0])*256+(int)(Uart2movetoRes[1]);
//        int Iweizhi = Ibianmaqi/10;                          //?????
      if(weizhiTo - Iweizhi>0)
      {/*
        if(((P2IN&LS1_LA)==LS1_LA)||((P1IN&LS2_LA)==LS2_LA))//�ַ�������ͷ����λ��
        {
          movetoZ_flag = 0;
          movetoZ_status = 1;                                                //1����Ϊ�ƶ���λ   ��
          //���ֹͣ
          P1OUT |= LA_IN1 ;//��
          P1OUT |= LA_IN2 ;//��
          TA0CCR2=0;    
        }*/
//          UpDownFlag = 1;                                //��ʾ��Ҫ����
        if(weizhiTo - Iweizhi < 2)                     //5Ϊ��������ǰ����������
        {
                                                       //**********ֹͣ����
          P1OUT |= LA_IN1 ;//��
          P1OUT |= LA_IN2 ;//��
          TA0CCR2=0;
          movetoZ_status = 1;                           //λ��ָ��״̬��0,����Ĭ��״̬  ��1������λ��   ��2���������˶�����ָ����   ��3�����������ϵ�������ʧ��   ��
          movetoZ_flag = 0;                           //���ٽ���λ���ƶ�ָ���ִ��            
        }
        else //if(weizhiTo - Iweizhi >= 2 && weizhiTo - Iweizhi <= 20)                     //����
        {
                                                       //�������ٶ��趨ֵ��ռ�ձȣ���ϵ
            P1OUT |= LA_IN1 ;//��
            P1OUT &= ~LA_IN2 ;//��
        
            TA0CCR2=1000;
        
        }/*
        else if(weizhiTo - Iweizhi > 20 && weizhiTo - Iweizhi <= 50)                     //����
        {
                                                       //�ٶ��趨ֵ��ϵ
            P1OUT |= LA_IN1 ;//��
            P1OUT &= ~LA_IN2 ;//��
            TA0CCR2=1000;
        
        }
        else
        {
                                                       //���е�����ٶ��趨ֵ   
            P1OUT |= LA_IN1 ;//��
            P1OUT &= ~LA_IN2 ;//��
            TA0CCR2=2000;
        }
        */
      }
      else if(weizhiTo - Iweizhi <= 0)
      {
//          UpDownFlag = 0;                                //0��ʾ��Ҫ���� ��1��ʾ��Ҫ����
        if(weizhiTo - Iweizhi > -2)                     //5Ϊ��������ǰ����������
        {
                                                       //**********ֹͣ����
            P1OUT |= LA_IN1 ;//��
            P1OUT |= LA_IN2 ;//��
            TA0CCR2=0;
            movetoZ_status = 1;                           //����λ��            
            movetoZ_flag = 0; 
        }
        else //if(weizhiTo - Iweizhi <= -2 && weizhiTo - Iweizhi >= -20)                     //����
        {
                                                       //�������ٶ��趨ֵ��ռ�ձȣ���ϵ
            P1OUT &= ~LA_IN1 ;//��
            P1OUT |= LA_IN2 ;//��
            TA0CCR2=1000;
        
        }/*
        else if(weizhiTo - Iweizhi < -20 && weizhiTo - Iweizhi >= -50)                     //����
        {
                                                       //�ٶ��趨ֵ��ϵ
            P1OUT &= ~LA_IN1 ;//��
            P1OUT |= LA_IN2 ;//��
            TA0CCR2=1000;
        
        }
        else
        {
                                                       //���е�����ٶ��趨ֵ 
            P1OUT &= ~LA_IN1 ;//��
            P1OUT |= LA_IN2 ;//��
            TA0CCR2=2000;
        }   
 */       
      }          
    }
                                            
  }
}

/*******************************************************************************
�綯�Ƹ��ƶ���ָ��λ�� �������ָ��
*******************************************************************************/
void MoveToZ_END(void)
{
    P1OUT |= LA_IN1 ;//��
    P1OUT |= LA_IN2 ;//��
    TA0CCR2=0;
    //
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x05 ,0x90};
    Uart2_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
}
/*******************************************************************************
�ƶ���ָ̨��
*******************************************************************************/
void InfraPtzRoll ()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x06 ,0xc3};
    Uart2_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
    
    if(Uart2ptzRollRes[0]==0)//��
    {
        yuntaiRollUp=1;
        yuntaiRollDown=0;
        yuntaiRollLeft=0;
        yuntaiRollRight=0;
    }
    else if(Uart2ptzRollRes[0]==1)//��
    {
        yuntaiRollUp=0;
        yuntaiRollDown=1;
        yuntaiRollLeft=0;
        yuntaiRollRight=0;
    }
    else if(Uart2ptzRollRes[0]==2)//��
    {
        yuntaiRollUp=0;
        yuntaiRollDown=0;
        yuntaiRollLeft=1;
        yuntaiRollRight=0;
    }
    else if(Uart2ptzRollRes[0]==3)//��
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
    //ת����̨
        
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
ֹͣ�ƶ���ָ̨��
*******************************************************************************/
void InfraPtzStop ()
{
    unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x07 ,0xF2};
    Uart2_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
    
    yuntaiRollUp=0;
    yuntaiRollDown=0;
    yuntaiRollLeft=0;
    yuntaiRollRight=0;
}
/*******************************************************************************
�ƶ���̨��ָ��λ��
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
    Uart2_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
}
/*******************************************************************************
��Сʱ��Ƭ
*******************************************************************************/
void MinTimeslice()//20msʱ��Ƭ
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
ָ�����
*******************************************************************************/
void InstructProcess(void)
{
    if(Uart2InstructFlag==0x01)//������ָ��
    {
        Uart2InstructFlag=0x00;//���־
        switch (Uart2InstructNum)
        {
        case 0x01://״̬��ѯָ��
          DetectState();
          break;
          
        case 0x02://�ƶ��綯�Ƹ�ָ��
          Move_Z();
          break;
          
        case 0x03://ֹͣ�ƶ��綯�Ƹ�ָ��
          Stop_Z();
          break;
     
        case 0x04://�ƶ��綯�Ƹ˵�ָ��λ��ָ��
          {
              unsigned char buffer[6]={0xfe ,0x04 ,0x01 ,0x03 ,0x04 ,0xa1};
              Uart2_send(buffer,6); //�Ȼظ����أ���ʾ���յ�ָ����
              
              movetoZ_status=0;
              movetoZ_flag = 1;
          }
          break;
          
        case 0x05://�ƶ���ָ��λ����ɣ��������ָ��
          {
              movetoZ_flag = 0;
              movetoZ_status = 0;                        //λ��ָ����� ǿ�ƻָ�
              //ͬʱ���ֹͣ  
              MoveToZ_END();
          }
          break;
        case 0x06://�ƶ���̨
          InfraPtzRoll ();
          break;
          
        case 0x07://ֹͣ�ƶ���̨
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
��������������ȡ����������
*******************************************************************************/
unsigned char DetectObstacle(void)
{
    if(Uart2InstructFlag==0x00)//û������ָ��
    {
        unsigned char a=0x01;
        Uart1_send(&a,1);//���ڷ�����������ָ��
        Init_Timer0_B7();//��10ms��ʱ��
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
        if(Uart1InstructFlag == 1)//�����������ݴ�����
        {
            if(isObstacle==0x01)//�����ϰ���
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
����ʱ����5�����������ϴ����������������������ʱ����100ms
*******************************************************************************/
void DetectSensor(void)
{
    if(Uart2InstructFlag != 0x01)//û������ָ��
    {
        MinTimeslice();//20msʱ��Ƭ��������ʱ20ms��
        detectNum++;
        RollYunTai();//����ת����̨
        switch(detectNum)
        {
          case 1:
          {
            unsigned int dist=distData[0]*256+distData[1];
            if((dist>250)&&(dist<600))
            {
              isObstacleFront4=0x01;//���ϰ���
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront4=0x00;//���ϰ���
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                P8OUT |= USC1;//ѡ��ڶ��������������� ǰ����  �й��� ͷ������
                P8OUT &= ~USC2;
                P8OUT &= ~USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//���ڷ�����������ָ��
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
              isObstacleFront1=0x01;//���ϰ���
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront1=0x00;//���ϰ���
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x03)
            {
                
                P8OUT &= ~USC1;//ѡ���һ�������������� ǰ����
                P8OUT &= ~USC2;
                P8OUT &= ~USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//���ڷ�����������ָ��
                
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
              isObstacleBelow=0x01;//���ϰ���
            }
            else
              isObstacleBelow=0x00;//���ϰ���
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                
                P8OUT &= ~USC1;//ѡ������������������� ǰ���� �����
                P8OUT &= ~USC2;
                P8OUT |= USC3;
                
                unsigned char a=0x01;
                Uart1_send(&a,1);//���ڷ�����������ָ��
                
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
              isObstacleFront2=0x01;//���ϰ���
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront2=0x00;//���ϰ���
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                
                P8OUT &= ~USC1;//ѡ����ĸ������������� D ǰ����
                P8OUT |= USC2;
                P8OUT |= USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//���ڷ�����������ָ��
                
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
              isObstacleFront3=0x01;//���ϰ���
              isObstacleFront=0x01;
              //isObstacleFrontCount++;
            }
            else
              isObstacleFront3=0x00;//���ϰ���
            
            distData[0]=0;
            distData[1]=0;
            
            if(moveDir==0x01)
            {
                
                P8OUT |= USC1;//ѡ������������������ ǰ����
                P8OUT &= ~USC2;
                P8OUT |= USC3;
                unsigned char a=0x01;
                Uart1_send(&a,1);//���ڷ�����������ָ��
                
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