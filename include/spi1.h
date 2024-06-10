#if !defined(SPI1_H)
#define SPI1_H

#include <stm32f410rx.h>
#include <stm32f4xx.h>
#include <user_types.h>
#include <user_functions.h>

static void spi1_clock_init();
static void spi1_pin_init();
static void spi1_config();
void spi1_init();
void spi1_transmit(uint8_t *data, uint32_t size);
void spi1_receive(uint8_t *data, uint32_t size, bool dummy_data);
void cs_enable();
void cs_disable();

#endif // SPI1_H
