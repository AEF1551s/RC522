#include <gpio.h>

static void clock_init()
{
    // AHB1 clock port B enable
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
}

static void pin_init()
{
    // PB11 input
    CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODER11);
    // PB11 internal pulldown
    SET_BIT(GPIOB->PUPDR, 0x2UL << GPIO_PUPDR_PUPD11_Pos);

    //PB13 LED general output
    SET_BIT(GPIOB->MODER, GPIO_MODER_MODE13_0);
}

static void lock()
{
    uint32_t lock_mask_on = GPIO_LCKR_LCKK | GPIO_LCKR_LCK11;
    uint32_t lock_mask_off = GPIO_LCKR_LCK11;
    uint32_t gpiob_lckr = 0;
    uint32_t lock_active = 0;

    // Lock key write sequence = WR 1+; WR 0+; WR 1+; RD LCKR; RD LCKR[16] = 1(optional);
    WRITE_REG(GPIOB->LCKR, lock_mask_on);
    WRITE_REG(GPIOB->LCKR, lock_mask_off);
    gpiob_lckr = READ_REG(GPIOB->LCKR);

    lock_active = (GPIOB->LCKR, GPIO_LCKR_LCKK); // true/false = 1/0;
}

void gpio_init()
{
    clock_init();
    pin_init();
    lock();
}
