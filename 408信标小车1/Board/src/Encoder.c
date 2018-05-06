#include "include.h"
#include "common.h"




void Encoder_init()
{
    printf("\n*****FTM 正交解码 测试*****\n");

    ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）A12 A13右
    ftm_quad_init(FTM2);                                    //A10  A11左

    pit_init_ms(PIT0, 500);                                 //初始化PIT0，定时时间为： 1000ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
    NVIC_SetPriority(PIT0_IRQn,1);         //配置优先级
    enable_irq (PIT0_IRQn);                                 //使能PIT0中断

 //   while(1);
}

/*!
 *  @brief      PIT0中断服务函数
 *  @since      v5.0
 */
void PIT0_IRQHandler(void)
{

    int16 right;
    int16 left;
    right = ftm_quad_get(FTM1);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
    left = ftm_quad_get(FTM2);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
    ftm_quad_clean(FTM1);
    ftm_quad_clean(FTM2);

    if(left>=0)
    {
        printf("\n左轮反转：%d",left);
    }
    else
    {
        printf("\n左轮正转：%d",-left);
    }
    if(right>=0)
    {
        printf("\n右轮正转：%d",right);
    }
    else
    {
        printf("\n右轮反转：%d",-right);
    }

    PIT_Flag_Clear(PIT0);       //清中断标志位
}
