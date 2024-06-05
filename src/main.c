#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <startup.c>
#include <stdbool.h>

//TODO: Add SPI, GPIO, TIMER headers.

void exti_interrupt_init()
{
    // EXTI Control Register 3, Set IRQ for PA11;
    SET_BIT(SYSCFG->EXTICR[3], SYSCFG_EXTICR3_EXTI11_PA << SYSCFG_EXTICR3_EXTI11_Pos);

    // TODO: SPI interupts
    // TODO: TIMER interupts
}

void gpio_init()
{
    // PA11 input, pulldown resistor
}

int main()
{
    while (true)
    {

    }
    return 0;
}