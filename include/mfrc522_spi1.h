#if !defined(MFRC522_SPI1_H)
#define MFRC522_SPI1_H

#include <mfrc522.h>
#include <spi1.h>

void write_register();
void read_register();

void mfrc522_read_data_byte(PCD_Register addr,uint8_t *data);
#endif // MFRC522_SPI1_H
