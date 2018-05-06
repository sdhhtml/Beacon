#include "common.h"
#include "include.h"
#include "math.h"

uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������

uint8 img_buf[60][80]; 
uint8 Flag_50ms = 0,Flag_5ms = 0,Flag_10ms=0,Flag_20ms=0,Flag_100ms=0,Flag_200ms=0;
int rtdis,mid_3,m;
extern PD pd_var;

extern sPID SPID;
//��������
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void PIT1_IRQHandler(void);
void  main(void)
{
    int i=0,j=0,k=0;
    int mid_1=0,mid_2=0;
    gpio_init(PTB20,GPO,1);
    gpio_init(PTB21,GPO,1);
    gpio_init(PTB22,GPO,1);
    gpio_init(PTB23,GPO,1);
    
    camera_init(imgbuff);                                  //�����趨  imgbuff Ϊ�ɼ�������������������
    Motor_init();
    PD_init(&pd_var);      //���������ʼ��
    Steering_init();
    spid_init(&SPID);            //���PID�����ĳ�ʼ��
    //�����жϷ�����
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����PORTA���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����DMA0���жϷ�����Ϊ DMA0_IRQHandler
    
    pit_init_ms(PIT1,1);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 1000ms
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    enable_irq(PIT1_IRQn); 
 while(1)
    {
        camera_get_img();                                   //����ͷ��ȡͼ��

       img_extract((uint8 *)img_buf,(uint8 *)imgbuff, CAMERA_W*CAMERA_H/8);        //��ѹΪ�Ҷ�ͼ�񣬷��㷢�͵���λ����

       for(i=5;i<40;i++)
        { 
          for(j=0;j<79;j++){
          if(img_buf[i][j]!=img_buf[i][j+1])
          {
           // printf("�ҵ�����㣺%d\r\n",j);
           k++;
           mid_1+=j;

          }             
        }
        }
       mid_2=mid_1/k;
//       printf("���ߵĺ�����Ϊ��%d\r\n",mid_2);
       mid_1=0;k=0;

       mid_3=mid_2;
      rtdis=Ultrasonic();
 }
}

/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}
void PIT1_IRQHandler(void)
{
    Flag_5ms++;
    Flag_10ms++;
    Flag_20ms++;
    Flag_50ms++;
    Flag_100ms++;
    Flag_200ms++;
    if(Flag_20ms==20){
      

       if((rtdis<505)&&(rtdis!=0))m=1;
       else m=0;
    }
    if(m==0)   {
                        if(Flag_5ms==5)
                        {
                            ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000);
                            ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                            ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                            ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,2000);
                        }
                        if(Flag_10ms==10)
                        {
                          //printf("���ߵĺ�����Ϊ��%d\r\n",mid_3);
                          PD_calculation(&pd_var,mid_3);     
                        }
                        if(Flag_50ms==50)
                        {
                        gpio_turn(PTB20);
                        gpio_turn(PTB21);
                        gpio_turn(PTB22);
                        gpio_turn(PTB23);
                        }
                        }
    else if(m==1){
                          if(Flag_5ms==5)
                    {
                        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,2000);
                    }
                    if(Flag_10ms==10)
                    {
                      ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,2340);   
                    }
                    if(Flag_50ms==50)
                    {
                    gpio_turn(PTB20);
                    gpio_turn(PTB21);
                    gpio_turn(PTB22);
                    gpio_turn(PTB23);
                    }
    }
    
if(Flag_5ms==5)Flag_5ms=0;
if(Flag_10ms==10)Flag_10ms=0;
if(Flag_20ms==20)Flag_20ms=0;
if(Flag_50ms==50)Flag_50ms=0;
if(Flag_100ms==100)Flag_100ms=0; 
if(Flag_200ms==200)Flag_200ms=0;  
PIT_Flag_Clear(PIT1);      //���жϱ�־λ
}
