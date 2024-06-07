#include <mfrc522_spi1.h>

void mfrc522_read_data_byte(PCD_Register addr, uint8_t *data)
{ // addr - adress byte, *data - for storing received data
    cs_enable();

    uint8_t tx_byte = 0x00;
    tx_byte = 0x1 << 7; // READ bit
    tx_byte |= addr;    // addr is already offset by 1 in mfrc522.h

    // Send adress byte we want to read
    spi1_transmit(&tx_byte, 1);

    // Dummy data is in implemented in spi1_receive(). Send dummy byte, receiving is offset by 1 sent byte.
    // End of Tx, now read received byte
    spi1_receive(data, 1);

    cs_disable();
}