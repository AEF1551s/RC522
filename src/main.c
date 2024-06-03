#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <startup.c>
#include <stdbool.h>
#include <spi1.h>

int main()
{

    // Clock enable PA
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    // Set PA9 as output
    SET_BIT(GPIOA->MODER, 0b01 << 10);

    // Infinite loop
    while (true)
    {
        SET_BIT(GPIOA->BSRR, 1U << 5);

        for (volatile uint32_t i = 0; i < 100000; i++)
            ;

        SET_BIT(GPIOA->BSRR, 1U << 21);

        for (volatile uint32_t i = 0; i < 100000; i++)
            ;
    }

    return 0;
}