#ifndef _DIFSPEED_H_
#define _DIFSPEED_H_

//PID差速输出的最大值
#define Diff_Output_Max  2000

void PD_Diff_init();
int16 PD_Diff_calculation(uint8 );

typedef struct PD_Diff {
    
    float Diff_Kp;
    float Diff_Kd;
    
    int16 error;
    int16 last_error;
    
    int16 Diff_output;           //差速输出；
    int16 PID_output;            //电机普通速度输出；
}PD_Diff;


#endif
