#include "spi1.h"

pin_struct_TypeDef cs_pin;
pin_struct_TypeDef sck_pin;
pin_struct_TypeDef miso_pin;
pin_struct_TypeDef mosi_pin;

static void spi1_clock_init()
{
    // GPIOA clock enable for slave select pin
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    //GPIOB clock enable for SPI1 pins
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
    // SPI1 clock enable
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
}

static void spi1_pin_init()
{
    /******SPI1 pins******/
    cs_pin = pin_setup(GPIOA, PIN9, OUTPUT);
    sck_pin = pin_setup(GPIOB, PIN3, ALTERNATE);
    miso_pin = pin_setup(GPIOB, PIN4, ALTERNATE);
    mosi_pin = pin_setup(GPIOB, PIN5, ALTERNATE);

    // Set AF5
    SET_BIT(GPIOB->AFR[0], GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_2); //SPI1_SCK
    SET_BIT(GPIOB->AFR[0], GPIO_AFRL_AFRL4_0 | GPIO_AFRL_AFRL4_2); //SPI1_MISO
    SET_BIT(GPIOB->AFR[0], GPIO_AFRL_AFRL5_0 | GPIO_AFRL_AFRL5_2); //SPI1_MOSI
    /*********************/
}

static void spi1_config()
{

    // Set baud-rate control to f(pckl)/4 = 4MHz; 0b001
    SET_BIT(SPI1->CR1, SPI_CR1_BR_0);
    CLEAR_BIT(SPI1->CR1, SPI_CR1_BR_1);
    CLEAR_BIT(SPI1->CR1, SPI_CR1_BR_2);
    // SPI1->CR1 &= ~(1U << 4);
    // SPI1->CR1 &= ~(1U << 5);

    // Set CPOL and CPHA to 1 (from datasheet)
    SET_BIT(SPI1->CR1, SPI_CR1_CPOL);
    SET_BIT(SPI1->CR1, SPI_CR1_CPHA);
    // Set to full-duplex mode
    CLEAR_BIT(SPI1->CR1, SPI_CR1_RXONLY);
    // Set MSB first
    CLEAR_BIT(SPI1->CR1, SPI_CR1_LSBFIRST);
    // Set SPI to be in master mode
    SET_BIT(SPI1->CR1, SPI_CR1_MSTR);
    // Set data frame to 8 bit mode
    CLEAR_BIT(SPI1->CR1, SPI_CR1_DFF);
    // Set to software(internal) slave select
    SET_BIT(SPI1->CR1, SPI_CR1_SSM);
    // Set slave select
    SET_BIT(SPI1->CR1, SPI_CR1_SSI);
    // Enable SPI peripheral
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
}

void spi1_init()
{
    spi1_clock_init();
    spi1_pin_init();
    spi1_config();
}

void spi1_transmit(uint8_t *data, uint32_t size)
{
    uint32_t i = 0;
    volatile uint8_t temp;
    while (i < size)
    {
        // Wait until TXE is set
        while (!READ_BIT(SPI1->SR, SPI_SR_TXE))
            ;

        // Write to the data register
        WRITE_REG(SPI1->DR, data[i]);

        i++;
    }
    // Wait unitl TXE is set
    while (!READ_BIT(SPI1->SR, SPI_SR_TXE))
        ;
    // Wait for BUSY to reset
    while (READ_BIT(SPI1->SR, SPI_SR_BSY))
        ;

    // Clear OVR overrun flag
    temp = SPI1->DR;
    temp = SPI1->SR;
}

void spi1_receive(uint8_t *data, uint32_t size)
{
    while (size)
    {
        // Send dummy data
        SPI1->DR = 0;

        // Wait unitl RXNE is set
        while (!READ_BIT(SPI1->SR, SPI_SR_RXNE))
            ;

        // Read data register
        *data = (SPI1->DR);
        data++;
        size--;
    }
}

void cs_enable()
{
    // SET_BIT(SPI1->CR1, SPI_CR1_SSI);
    GPIOA->ODR &= ~(1U << 9);
    // digital_write(cs_pin, LOW);
    // Also with BSRR
}

void cs_disable()
{
    // CLEAR_BIT(SPI1->CR1, SPI_CR1_SSI);
    GPIOA->ODR |= (1U << 9);
    // digital_write(cs_pin, HIGH);
    // Also with BSRR
}