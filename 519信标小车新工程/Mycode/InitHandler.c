#include "common.h"
#include "include.h"
extern uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组


uint8 Flag_50ms = 0,Flag_5ms = 0,Flag_10ms=0,Flag_20ms=0,Flag_100ms=0,Flag_200ms=0;
extern uint16 Flag_500ms=0;
extern int rtdis,m;
extern int mid_3,mid_second,mid_last1,mid_now,mid_last2,mid_last3,mid_last4,mid_last5,mid_last6,mid_input;
extern PD pd_var;
extern sPID SPID;
extern float Kp_motor;
extern float Proportion_1;
extern float Derivative_1;
/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
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
    Flag_500ms++;

    if(m==0)   {
      
      //    if(mid_second<5){
                            if(Flag_5ms==5)
                            {
                                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,2000);
                            }
                            if(Flag_10ms==10)
                            {
                              //printf("中线的横坐标为：%d\r\n",mid_3);
                              PD_calculation(&pd_var,mid_input);     
                            }
                            if(Flag_500ms==500)
                            {                              

                            }                    
  //      }       
/*          if(mid_second>=5){
                            if(Flag_5ms==5)
                            {
                                ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1000);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                                ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1500);
                            }
                            if(Flag_10ms==10)
                            {
                              //printf("中线的横坐标为：%d\r\n",mid_3);
                             ftm_pwm_duty(S3010_FTM,S3010_CH,2340); 
                            }
                            if(Flag_500ms==500)
                            {                              
                            gpio_turn(PTB20);
                            gpio_turn(PTB21);
                            gpio_turn(PTB22);
                            gpio_turn(PTB23);
                            }
                    
              }*/
}
    if(m==1){
                          if(Flag_5ms==5)
                    {
                        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,1500);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,1500);
                    }
                    if(Flag_10ms==10)
                    {
                     // ftm_pwm_duty(S3010_FTM, S3010_CH,2340);  
                      ftm_pwm_duty(S3010_FTM,S3010_CH,2300); 
                    }
                    if(Flag_500ms==500)
                    {

                    }
    }
  
if(Flag_5ms==5)Flag_5ms=0;
if(Flag_10ms==10)Flag_10ms=0;
if(Flag_20ms==20)Flag_20ms=0;
if(Flag_50ms==50)Flag_50ms=0;
if(Flag_100ms==100)Flag_100ms=0; 
if(Flag_200ms==200)Flag_200ms=0;  
if(Flag_500ms==500)Flag_500ms=0;
PIT_Flag_Clear(PIT1);      //清中断标志位
}
void PORTD_IRQHandler(void)
{


  //  PORT_FUNC(D,11,key_handler1);
    PORT_FUNC(D,12,key_handler2);
    PORT_FUNC(D,13,key_handler3);
    PORT_FUNC(D,14,key_handler4);

}


void key_handler2(void)
{
   //j+=1;gpio_turn(PTB22);if(j>4)j=1;
}
void key_handler3(void)
{
    disable_irq(PIT1_IRQn) ;
                        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
                        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
}
void key_handler4(void)
{
    enable_irq(PIT1_IRQn); 
}

void PORTA_IRQHandler_KEY(void)
{


    PORT_FUNC(A,5,ZKkey5);//5
    PORT_FUNC(A,7,ZKkey4);//4
    PORT_FUNC(A,8,ZKkey2);//2
    PORT_FUNC(A,9,ZKkey1);//1
}
void PORTE_IRQHandler_KEY(void)
{


    PORT_FUNC(E,25,ZKkey3);//3
    PORT_FUNC(E,27,ZKkey6);//6

}
void ZKkey1(void)
{
  while(!gpio_get(PTA9));
  Proportion_1+=0.1;
  DisplayFloat((Proportion_1*1000),66,0);
  PD_init(&pd_var);      //舵机参数初始化
  gpio_turn (PTB20);
}
void ZKkey2(void)
{
  while(!gpio_get(PTA8));
  Proportion_1-=0.1;
  DisplayFloat((Proportion_1*1000),66,0);
  PD_init(&pd_var);      //舵机参数初始化
  gpio_turn (PTB23);
}
void ZKkey3(void)
{
  while(!gpio_get(PTE25));
  Derivative_1+=0.1;
  DisplayFloat((Derivative_1*1000),66,2);
  PD_init(&pd_var);      //舵机参数初始化
  gpio_turn (PTB20);
}
void ZKkey4(void)
{
  while(!gpio_get(PTA7));
  Derivative_1-=0.1;
  DisplayFloat((Derivative_1*1000),66,2);
  PD_init(&pd_var);      //舵机参数初始化
  gpio_turn (PTB23);
}
void ZKkey5(void)
{
  while(!gpio_get(PTA5));
  Kp_motor+=0.1;
  DisplayFloat((Kp_motor*1000),66,4);
  spid_init(&SPID);            //电机PID参数的初始化
  gpio_turn (PTB20);
}
void ZKkey6(void)
{
  while(!gpio_get(PTE27));
  Kp_motor-=0.1;
  DisplayFloat((Kp_motor*1000),66,4);
  spid_init(&SPID);            //电机PID参数的初始化
  gpio_turn (PTB23);
}