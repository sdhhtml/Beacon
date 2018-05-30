#include "common.h"
#include "include.h"
//int16 l_result,r_result,l_error,r_error;
//int16 result,l_output,r_output;                            //���ҵ��ת��ƽ��ֵ
float motor_increment = 0;                                 //����ʽPID���������
float kp = 0;                                    //��ʼʱ�̸���һ�����kpֵ��
int16 motor_duty = 0;
//���PID����������
sPID SPID;
uint8 Mid_error = 0;                             //�����ٶȿ�������ƫ��
extern float Kp_motor;
extern PD_Diff PD_diff;
void Motor_init()
{
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,5000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,5000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,5000);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,5000);      //��ʼ�� ��� PWM
}
void Motor_test()
{
        int i = 3000;
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,i);
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,i);
}

//΢������PID�Ĳ���
void spid_init(sPID *spid)                           //�ٶ�PID�ĳ�ʼ��
{
    spid->sp_motor = 0;                                            //����趨ֵ
  
    spid->Kp_motor = 10;                                                 //Kp����
    spid->Ki_motor = 0;                                                 //Ki����
    spid->Kd_motor = 0;                                                 //Kd����
    kp = spid->Kp_motor;
    spid->error = 0;                                                   //����ƫ��
    spid->last_error = 0;                                            //��һ��ƫ��
    spid->pre_error = 0;                                             //���ϴ�ƫ��
    
    spid->output = 0;
    spid->last_output = 0;
    spid->pre_output = 0;
    
    spid->PD_Diff_output = 0;                                      //�������Ϊ0
} 

//΢�����е�PID�㷨
void pid_calculation(sPID *spid,int8 Mid,int8 vertical)
{
  //  spid->sp_motor=3800-vertical*60;
  spid->sp_motor=2000;
 //   if(spid->sp_motor<1800)spid->sp_motor=1800;
    spid->l_result=-(ftm_quad_get(FTM2)*76);
    spid->r_result=(ftm_quad_get(FTM1)*76);
    ftm_quad_clean(FTM1);
    ftm_quad_clean(FTM2);
    spid->result=(spid->l_result+spid->r_result)/2;
    
    spid->l_error = spid->sp_motor-spid->l_result;
    spid->r_error = spid->sp_motor-spid->r_result;
    spid->r_output = spid->r_output+spid->Kp_motor*spid->r_error;
    spid->l_output = spid->l_output+spid->Kp_motor*spid->l_error;
//    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,spid->l_output);
    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000);
    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,2097);
//    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,spid->r_output);
 //   printf("\n������ת��%d",l_result);
//    printf("\n������ת��%d",r_result);    
 //   result=(l_result+r_result)/2;
#if 0   
    //�����޷�
    spid->error = spid->sp_motor - result;                        //�趨ֵ-����ֵ 
    if(spid->error > Integration_Trh)  
        motor_increment = spid->Kp_motor * (spid->error  - spid->last_error)
                        + spid->Kd_motor * (spid->output - 2 * spid->last_output + spid->pre_output) 
                        - spid->Kd_motor * (spid->output - spid->last_output);
    else
        motor_increment = spid->Kp_motor * (spid->error  - spid->last_error)
                        + spid->Ki_motor * spid->error
                        + spid->Kd_motor * (spid->output - 2 * spid->last_output + spid->pre_output) 
                        - spid->Kd_motor * (spid->output - spid->last_output);
    motor_duty = (int16)((float)motor_duty + motor_increment);
    spid->output = motor_duty;                                       //����޷�����
    
 //   spid->output = (int16)((float)spid->last_output + motor_increment);
    
                                     
            //��������ֵ
    spid ->pre_error = spid ->last_error;
    spid ->last_error = spid ->error;
    spid ->pre_output = spid->last_output;
    spid ->last_output = spid->output;
          
//    result[0] = 0;
//    result[1] = 0;

    if(spid->output >3000)     
        spid->output = 3000;     
    if(spid->output < -3000)      
        spid->output = -3000;      

        spid->PD_Diff_output = PD_Diff_calculation(Mid);

    if(spid->output - spid->PD_Diff_output >= 0 && spid->output + spid->PD_Diff_output >= 0)                                         //��ת
    {
         //����
         ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,              0);
         ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,   spid->output - spid->PD_Diff_output);
         //�ҵ��
         ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,   spid->output + spid->PD_Diff_output);
         ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,              0);
    }
    else if(spid->output - spid->PD_Diff_output >= 0 && spid->output + spid->PD_Diff_output <= 0)                                         //��ת
    {    
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,                0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,     spid->output - spid->PD_Diff_output);
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,                0);      
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,    -(spid->output + spid->PD_Diff_output));
    }
    else if(spid->output - spid->PD_Diff_output <= 0 && spid->output + spid->PD_Diff_output >= 0)                                         //��ת
    {    
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,      -(spid->output - spid->PD_Diff_output));          
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,                0);
        //
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,      spid->output + spid->PD_Diff_output);      
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,                0);
    }
    else
    {    
        //
        ftm_pwm_duty(FTM0, MOTOR2_PWM,     -(spid->output - spid->PD_Diff_output)); 
        ftm_pwm_duty(FTM0, MOTOR1_PWM,                0);
        //
        ftm_pwm_duty(FTM0, MOTOR4_PWM,                0);      
        ftm_pwm_duty(FTM0, MOTOR3_PWM,    -(spid->output + spid->PD_Diff_output));
    }
#endif
//��������ʾ�����������ʾ
    

    
}

