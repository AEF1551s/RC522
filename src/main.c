#include <stm32f410rx.h>
#include <stm32f4xx.h>
// #include <startup.c>
#include <stdbool.h>
#include <gpio.h>
#include <exti.h>

// TODO: Add SPI, GPIO, TIMER headers.

int main()
{

    gpio_init();
    exti_init();

    while (true)
    {
        
        if (READ_BIT(GPIOB->IDR, GPIO_IDR_IDR_11))
        {

            // ON LED
            SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS13);

            // wait
            for (volatile int i = 0; i < 1000000; i++)
                ;
            // OFF LED
            SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR13);
            // wait
            for (volatile int i = 0; i < 1000000; i++)
                ;
        }
    }

    return 0;
}

// Interrupts
void EXTI15_10_IRQHandler(void)
{
    // if interrupt happens on 11 pin
    if (READ_BIT(EXTI->PR, EXTI_PR_PR11))
    {
        // Clear pending bit by writing 1 per RefManual. reset interupt request
        SET_BIT(EXTI->PR, EXTI_PR_PR11);

        // ON LED
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS13);

        // wait
        for (volatile int i = 0; i < 1000000; i++)
            ;
        // OFF LED
        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR13);
    }
}