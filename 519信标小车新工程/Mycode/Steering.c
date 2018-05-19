#include "common.h"
#include "include.h"

extern uint8 Speed_UP,Speed_Down,Speed_Mid;

extern uint8 Speed_UP,Speed_Down,Speed_Mid;
float Proportion_1=6.0;
float Derivative_1=6.0;
//PD 变量定义
PD pd_var;  

void PD_init(PD *pdvar)
{
    pdvar->setpoint = 40;  
    pdvar->error = 0;
    pdvar->last_error = 0;
    pdvar->duty = 2140;
    
    pdvar->Proportion_1 = Proportion_1;
    pdvar->Derivative_1 = Derivative_1;
      
    pdvar->Proportion_2 = 8.0;
    pdvar->Derivative_2 = 8.0;
    
}

void Steering_init()
{
    ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,2140);      //初始化 舵机 PWM
}

/**************************************************/
/*                    PD参数的计算                */
/*************************************************/
void PD_calculation(PD *pdvar,uint8 Mid)
{

    pdvar->error = pdvar->setpoint - Mid;   
    
    
    //if(Speed_UP == 1 || Speed_Mid == 1)
    //{
        pd_var.duty = pdvar->Proportion_1 * pdvar->error
                    + pdvar->Derivative_1 * (pdvar->error - pdvar->last_error)
                    + DUTY_MID;
    
    //}
#if 0
    else if(Speed_Down == 1)
    {
        pd_var.duty = pdvar->Proportion_2 * pdvar->error
                    + pdvar->Derivative_2 * (pdvar->error - pdvar->last_error)
                    + DUTY_MID;
    }
#endif
    
    pdvar->last_error = pdvar->error;
    
   if(pd_var.duty >= DUTY_MAX)                                //舵机PID防止最高和最低溢出。
     pd_var.duty = DUTY_MAX;
   else if(pd_var.duty <= DUTY_MIN)
      pd_var.duty = DUTY_MIN; 
   ftm_pwm_duty(S3010_FTM,S3010_CH,(uint32)pd_var.duty); 

}

void Steering_test()
{
    int i;



        for(i = 2140;i<2210;i+=20)
        {
            ftm_pwm_duty(S3010_FTM, S3010_CH,i);
            DELAY_MS(1000);
            printf("%d\n",i);
            
        }

        for(;i>2020;i-=20)
        {
            ftm_pwm_duty(S3010_FTM, S3010_CH,i);
            DELAY_MS(1000);
            printf("%d\n",i);
            
        }    
    

}