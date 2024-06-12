#include <mfrc522_spi1.h>

static void read_data_byte(PCD_Register addr, uint8_t *data) {
    cs_enable();

    uint8_t addr_byte = 0x00;
    addr_byte = 0x1 << 7; // READ bit
    addr_byte |= addr;    // addr is already offset by 1 in mfrc522.h

    // Send address byte we want to read
    spi1_transmit(&addr_byte, 1);

    // Dummy data is in implemented in spi1_receive(). Send dummy byte, receiving is offset by 1 sent byte.
    // End of Tx, now read received byte
    spi1_receive(data, 1, true);

    cs_disable();
}

static void read_data_stream(PCD_Register *addr, uint8_t *data, uint32_t size) {
    // size 0 is invalid
    if (size == 0) {
        return;
    }

    cs_enable();

    uint32_t index = 0;

    while (index < size) {
        uint8_t addr_byte = 0x00;
        addr_byte = 0x1 << 7; // READ bit
        addr_byte |= *addr;   // addr is already offset by 1 in mfrc522.h

        // Send address byte we want to read
        spi1_transmit(&addr_byte, 1);

        // If first read op, don't read first rx data
        if (index != 0) {
            spi1_receive(data, 1, false);
        }

        index++;
        data++;
        addr++;
    }

    cs_disable();
}

static void write_register(PCD_Register addr, uint8_t *data) {
    // Send address byte, then send data byte to write into address
    cs_enable();

    uint8_t addr_byte = 0x00;
    addr_byte = 0x0 << 7; // WRITE bit
    addr_byte |= addr;    // addr is already offset by 1 in mfrc522.h

    uint8_t data_byte = *data;

    uint8_t addr_data[2] = {addr_byte, data_byte};
    // Send address byte we want to read
    spi1_transmit(addr_data, 2);

    cs_disable();
}

// static void write_register_stream(PCD_Register addr, uint8_t *data, uint32_t size) {
//     cs_enable();
//     spi1_transmit(data, size);
//     cs_disable();
// }

static void write_register_stream(PCD_Register addr, uint32_t sendLen, uint8_t *data) {
    // Send address byte, then send data byte to write into address
    cs_enable();

    uint8_t addr_byte = 0x00;
    addr_byte = 0x0 << 7; // WRITE bit
    addr_byte |= addr;    // addr is already offset by 1 in mfrc522.h
    // Send address
    spi1_transmit(&addr, 1);
    // Send data stream
    spi1_transmit(data, sendLen);

    cs_disable();
}

void PCD_read_register_aligned(PCD_Register reg, uint32_t size, uint8_t *data, uint8_t rxAlign) {
    if (size == 0)
        return;

    uint8_t address = 0x80 | reg; // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t index = 0;            // Index in values array.
    cs_enable();                  // Select slave
    size--;                       // One read is performed outside of the loop

    spi1_transmit(&address, 1); // Tell MFRC522 which address we want to read

    while (index < size) {
        spi1_transmit(&address, 1);   // Send same address
        spi1_receive(data, 1, false); // Read received previous FIFO address contents
        data++;
        index++;
    }
    spi1_receive(data, 1, true); // Send 0, and read final data.
    cs_disable();
}

void mfrc522_reset() {
    // After SoftReset command, soft power down is set to 0 after reset is complete
    // Power down - 0; RcvOff - 0
    write_register(CommandReg, PCD_SoftReset);

    uint8_t command_reg = 0x00;
    bool ready = false;
    while (!ready) {
        read_data_byte(CommandReg, &command_reg);

        if (!READ_BIT(command_reg, 1 << 4)) // If 0 then out of soft power state, and ready.
            ready = true;                   // Read PowerDown bit
    }
}

static void mfrc522_antenna_on() {
    uint8_t value;
    read_data_byte(TxControlReg, &value);
    if ((value & 0x03) != 0x03) {
        write_register(TxControlReg, value | 0x03);
    }
}

void mfrc522_init() {
    mfrc522_reset();

    write_register(TxModeReg, 0x00);
    write_register(RxModeReg, 0x00);
    // Reset ModWidthReg
    write_register(ModWidthReg, 0x26);

    write_register(TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    write_register(TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25µs.
    write_register(TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    write_register(TReloadRegL, 0xE8);

    write_register(TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    write_register(ModeReg, 0x3D);
    mfrc522_antenna_on();
}

int mfrc522_pcd_get_version() {
    // Get the MFRC522 firmware version
    uint8_t version;
    read_data_byte(VersionReg, &version);

    // When 0x00 or 0xFF is returned, communication probably failed
    if ((version == 0x00) || (version == 0xFF))
        return 0x00;
    return version;
}

void PCD_set_register_bit_mask(PCD_Register reg, uint8_t mask) {
    uint8_t tmp;
    read_data_byte(reg, &tmp);
    uint8_t data = tmp | mask;
    write_register(reg, &data);
}

void PCD_clear_register_bit_mask(PCD_Register reg, uint8_t mask) {
    uint8_t tmp;
    read_data_byte(reg, &tmp);
    write_register(reg, tmp & (~mask));
}

StatusCode PCD_communicate_with_picc(PCD_Command command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen,
                                     uint8_t *backData, uint8_t *backLen, uint8_t *validBits,
                                     uint8_t rxAlign, bool checkCRC) {
    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    write_register(CommandReg, PCD_Idle);         // Stop any active command.
    write_register(ComIrqReg, 0x7F);              // Clear all seven interrupt request bits
    write_register(FIFOLevelReg, 0x80);           // FlushBuffer = 1, FIFO initialization
    write_register_stream(FIFODataReg, sendLen, sendData); // Write sendData to the FIFO
    write_register(BitFramingReg, bitFraming);    // Bit adjustments
    write_register(CommandReg, (uint8_t)command);          // Execute the command
    if (command == PCD_Transceive) {
        PCD_set_register_bit_mask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete
    uint8_t n;
    uint8_t irqVal;
    uint8_t status1;
    uint8_t status2;
    uint16_t i;

    for (i = 2000; i > 0; i--) {
        read_data_byte(ComIrqReg, &irqVal);    // ComIrqReg[7..0]
        if (irqVal & waitIRq) {
            break;
        }
        if (irqVal & 0x01) {  // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }

    // Check for timeout
    if (i == 0) {
        return STATUS_TIMEOUT;
    }

    // Error handling
    uint8_t error;
    read_data_byte(ErrorReg, &error); // ErrorReg[7..0]
    if (error & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    // Handle back data if any
    if (backData && backLen) {
        read_data_byte(FIFOLevelReg, &n);       // Number of bytes in the FIFO
        if (n > *backLen) {
            return STATUS_NO_ROOM;
        }
        *backLen = n;                          // Store number of bytes returned
        read_data_stream(FIFODataReg, backData, n);  // Get received data from FIFO
    }

    if (validBits) {
        read_data_byte(ControlReg, &n);
        *validBits = n & 0x07;
    }

    // Handle collision errors
    if (error & 0x08) {  // CollErr
        return STATUS_COLLISION;
    }

    // Return status
    return STATUS_OK;
}

StatusCode PCD_transceive_data(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen,
                               uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {
    return PCD_communicate_with_picc(PCD_Transceive, 0x30, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

StatusCode PICC_REQA_or_WUPA(PICC_Command command, uint8_t *buffer_ATQA, uint8_t *buffer_size) {
    uint8_t validBits;
    uint8_t waitIRq;
    uint8_t backData[18];   // ATQA can be 2 bytes long
    uint8_t backLen;

    // Only used in PICC_HaltA, otherwise unused
    if (command == PICC_CMD_HLTA) {
        waitIRq = 0x10;
        validBits = 0;
    } else {
        waitIRq = 0x30;
        validBits = 7;   // According to ISO/IEC 14443-3 7.2.1
    }

    // Send the command
    StatusCode result = PCD_transceive_data(&command, 1, backData, &backLen, &validBits, 0, false);
    if (result == STATUS_OK) {
        // ATQA must be exactly 2 bytes. Anything else must be interpreted as error.
        if (backLen != 2 || validBits != 0) {
            result = STATUS_ERROR;
        }
    }

    // Write ATQA response
    if (buffer_ATQA && buffer_size) {
        if (*buffer_size < 2) {
            return STATUS_NO_ROOM;
        }
        *buffer_size = 2;
        buffer_ATQA[0] = backData[0];
        buffer_ATQA[1] = backData[1];
    }

    return result;
}

StatusCode PICC_request_A(uint8_t *buffer_ATQA, uint8_t *buffer_size) {
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, buffer_ATQA, buffer_size);
}

bool PICC_new_card_present() {
    uint8_t buffer_ATQA[2];
    uint8_t buffer_size = sizeof(buffer_ATQA);
    StatusCode result = PICC_request_A(buffer_ATQA, &buffer_size);

    return (result == STATUS_OK || result == STATUS_COLLISION);
}
