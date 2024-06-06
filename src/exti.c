#include <exti.h>

void exti_init()
{
    // SYSCFG controller clock enable
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

    // APB2 EXTI clock enable
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_EXTITEN);

    // EXTI Control Register 3, Set IRQ for PB11;
    SET_BIT(SYSCFG->EXTICR[2], SYSCFG_EXTICR3_EXTI11_PB);

    // Rising triger selection
    //  EXTI_RTSR TR11 == PB11
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR11);

    // Interrupt mask register
    // EXTI_IMR MR11 == PB11
    SET_BIT(EXTI->IMR, EXTI_IMR_MR11);

    // Enable interupts
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
}