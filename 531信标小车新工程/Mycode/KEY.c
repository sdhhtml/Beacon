#include "include.h"
#include "common.h"
void ZK_keyinit()
{
gpio_init(PTA5,GPI,1);//5
gpio_init(PTA7,GPI,1);//4
gpio_init(PTA8,GPI,1);//2
gpio_init(PTA9,GPI,1);//1
gpio_init(PTE25,GPI,1);//3
gpio_init(PTE27,GPI,1);//6

/*
gpio_init(PTB20,GPO,1);
gpio_init(PTB21,GPO,1);
gpio_init(PTB22,GPO,1);
gpio_init(PTB23,GPO,1);
*/
port_init(PTA5, ALT1 | IRQ_FALLING | PULLUP ); 
port_init(PTA7, ALT1 | IRQ_FALLING | PULLUP );
port_init(PTA8, ALT1 | IRQ_FALLING | PULLUP );
port_init(PTA9, ALT1 | IRQ_FALLING | PULLUP );
port_init(PTE25, ALT1 | IRQ_FALLING | PULLUP );
port_init(PTE27, ALT1 | IRQ_FALLING | PULLUP );

    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler_KEY);    //设置PORTA的中断服务函数为 PORTA_IRQHandler
    enable_irq (PORTA_IRQn); 
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler_KEY);    //设置PORTE的中断服务函数为 PORTE_IRQHandler
    enable_irq (PORTE_IRQn); 
}
