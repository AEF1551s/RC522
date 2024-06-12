#if !defined(MFRC522_SPI1_H)
#define MFRC522_SPI1_H

// From https://github.com/miguelbalboa/rfid official Arduino library

#include <mfrc522.h>
#include <spi1.h>
#include <stdbool.h>
#include <stdlib.h>

#define FIFO_SIZE 64 // For future use

// SPI functions
static void read_data_byte(PCD_Register addr, uint8_t *data);
static void read_data_stream(PCD_Register *addr, uint8_t *data, uint32_t size);
static void write_register(PCD_Register addr, uint8_t *data);
static void write_register_stream(PCD_Register addr, uint32_t size, uint8_t *data);
void PCD_read_register_aligned(PCD_Register reg, uint32_t size, uint8_t *data, uint8_t rxAlign);

// RFID device (PCD) functions
void mfrc522_reset();
static void mfrc522_antenna_on();
void mfrc522_init();
int mfrc522_pcd_get_version();
void PCD_set_register_bit_mask(PCD_Register reg, uint8_t mask);
void PCD_clear_register_bit_mask(PCD_Register reg, uint8_t mask);

// RFID card/tag (PICC) functions
StatusCode PCD_communicate_with_picc(PCD_Command command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen,
                                     uint8_t *backData, uint8_t *backLen, uint8_t *validBits,
                                     uint8_t rxAlign, bool checkCRC);
StatusCode PCD_transceive_data(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen,
                               uint8_t *validBits, uint8_t rxAlign, bool checkCRC);
StatusCode PICC_REQA_or_WUPA(PICC_Command command, uint8_t *buffer_ATQA, uint8_t *buffer_size);
StatusCode PICC_request_A(uint8_t *buffer_ATQA, uint8_t *buffer_size);
bool PICC_new_card_present();

#endif // MFRC522_SPI1_H
