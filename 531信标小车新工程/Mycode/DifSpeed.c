#include "common.h"
#include "DifSpeed.h"

//PD差速变量的声明
PD_Diff PD_diff;

void PD_Diff_init()
{
    PD_diff.Diff_Kp = 10;           //差速Kp值
    PD_diff.Diff_Kd = 0;            //差速Kd值
    
    PD_diff.error = 0;              //差速偏差值
    PD_diff.last_error = 0;         //差速上一次偏差值
    
    PD_diff.Diff_output = 0;        //差速输出值 
}

int16 PD_Diff_calculation(uint8 Mid)         //差速PID的计算--根据CCD中线值
{
    PD_diff.error = 40 - Mid;                 //得到偏差值
   //位置式PD计算
    PD_diff.Diff_output = (int16)(PD_diff.Diff_Kp * PD_diff.error
                                + PD_diff.Diff_Kd * (PD_diff.error - PD_diff.last_error));
    //更新数据值
    PD_diff.last_error = PD_diff.error;
    //限幅处理
    if(PD_diff.Diff_output >= Diff_Output_Max)   PD_diff.Diff_output = Diff_Output_Max;
    if(PD_diff.Diff_output <= -Diff_Output_Max)  PD_diff.Diff_output = -Diff_Output_Max;
    
    return PD_diff.Diff_output;
}
