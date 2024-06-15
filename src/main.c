#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <stdbool.h>
#include <gpio.h>
#include <exti.h>
#include <mfrc522.h>
#include <spi1.h>
#include <stdlib.h>

// Global variables
Uid current_uid;
//TODO: Add dynamic array functions for storing last 3 picc uids
// TODO: Add SPI, GPIO, TIMER headers.

int main()
{
    // Setup
    gpio_init();
    exti_init();
    spi1_init();
    mfrc522_init();
    
    // delay
    for (volatile uint32_t i = 0; i < 100000; i++)
        ;

    uint8_t version = mfrc522_pcd_get_version();

    // Loop
    while (true)
    {
    }

    return 0;
}

// Interrupts

// When button is pressed, PCD communicated with PICC to check if card is present
// Handler returns only when card is present
void EXTI15_10_IRQHandler(void)
{
    // if interrupt happens on 11 pin
    if (READ_BIT(EXTI->PR, EXTI_PR_PR11))
    {
        // Clear pending bit by writing 1 per RefManual. reset interupt request
        SET_BIT(EXTI->PR, EXTI_PR_PR11);

        while (true)
        {
            if (!PICC_new_card_present())
                return;

            if (!PICC_read_card_serial(&current_uid))
                return;
        }
    }
}