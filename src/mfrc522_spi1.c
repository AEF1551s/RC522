#include <mfrc522_spi1.h>

static void read_data_byte(PCD_Register addr, uint8_t *data)
{
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

static void read_data_stream(PCD_Register addr, uint8_t *data, uint32_t size)
{
    // size 0 is invalid
    if (size == 0)
    {
        return;
    }

    cs_enable();

    uint32_t index = 0;

    while (index < size)
    {
        uint8_t addr_byte = 0x00;
        addr_byte = 0x1 << 7; // READ bit
        addr_byte |= addr;   // addr is already offset by 1 in mfrc522.h

        // Send address byte we want to read
        spi1_transmit(&addr_byte, 1);

        // If first read op, don't read first rx data
        if (index != 0)
        {
            spi1_receive(data, 1, false);
        }

        index++;
        data++;
        addr++;
    }

    cs_disable();
}

static void write_register(PCD_Register addr, uint8_t *data)
{
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

static void write_register_stream(PCD_Register addr, uint32_t send_len, uint8_t *data)
{
    // Send address byte, then send data byte to write into address
    cs_enable();

    uint8_t addr_byte = 0x00;
    addr_byte = 0x0 << 7; // WRITE bit
    addr_byte |= addr;    // addr is already offset by 1 in mfrc522.h
    // Send address
    spi1_transmit(&addr_byte, 1);
    // Send data stream
    spi1_transmit(data, send_len);

    cs_disable();
}

void PCD_read_register_aligned(PCD_Register reg, uint32_t size, uint8_t *data, uint8_t rx_align)
{
    if (size == 0)
        return;

    uint8_t address = 0x80 | reg; // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t index = 0;            // Index in values array.
    cs_enable();                  // Select slave
    size--;                       // One read is performed outside of the loop

    spi1_transmit(&address, 1); // Tell MFRC522 which address we want to read

    while (index < size)
    {
        spi1_transmit(&address, 1);   // Send same address
        spi1_receive(data, 1, false); // Read received previous FIFO address contents
        data++;
        index++;
    }
    spi1_receive(data, 1, true); // Send 0, and read final data.
    cs_disable();
}

void write_PCD_command(PCD_Command cmd)
{
    PCD_Register reg = CommandReg;
    uint8_t data = (uint8_t)cmd;
    write_register(reg, &data);
}

void mfrc522_reset()
{
    // After SoftReset command, soft power down is set to 0 after reset is complete
    // Power down - 0; RcvOff - 0
    write_PCD_command(PCD_SoftReset);

    uint8_t command_reg = 0x00;
    bool ready = false;
    while (!ready)
    {
        read_data_byte(CommandReg, &command_reg);

        if (!READ_BIT(command_reg, 1 << 4)) // If 0 then out of soft power state, and ready.
            ready = true;                   // Read PowerDown bit
    }
}

static void mfrc522_antenna_on()
{
    uint8_t value;
    read_data_byte(TxControlReg, &value);
    if ((value & 0x03) != 0x03)
    {
        write_register(TxControlReg, &(uint8_t){value | 0x03});
    }
}

void mfrc522_init()
{
    mfrc522_reset();

    write_register(TxModeReg, &(uint8_t){0x00});
    write_register(RxModeReg, &(uint8_t){0x00});
    // Reset ModWidthReg
    write_register(ModWidthReg, &(uint8_t){0x26});

    write_register(TModeReg, &(uint8_t){0x80});      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    write_register(TPrescalerReg, &(uint8_t){0xA9}); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25µs.
    write_register(TReloadRegH, &(uint8_t){0x03});   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    write_register(TReloadRegL, &(uint8_t){0xE8});

    write_register(TxASKReg, &(uint8_t){0x40}); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    write_register(ModeReg, &(uint8_t){0x3D});
    mfrc522_antenna_on();
}

int mfrc522_pcd_get_version()
{
    // Get the MFRC522 firmware version
    uint8_t version;
    read_data_byte(VersionReg, &version);

    // When 0x00 or 0xFF is returned, communication probably failed
    if ((version == 0x00) || (version == 0xFF))
        return 0x00;
    return version;
}

void PCD_set_register_bit_mask(PCD_Register reg, uint8_t mask)
{
    uint8_t tmp;
    read_data_byte(reg, &tmp);
    uint8_t data = tmp | mask;
    write_register(reg, &data);
}

void PCD_clear_register_bit_mask(PCD_Register reg, uint8_t mask)
{
    uint8_t tmp;
    read_data_byte(reg, &tmp);
    uint8_t tmp_mask = tmp & (~mask);
    write_register(reg, &tmp_mask);
}

StatusCode PCD_communicate_with_picc(PCD_Command command, uint8_t wait_irq, uint8_t *send_data, uint8_t send_len,
                                     uint8_t *back_data, uint8_t *back_len, uint8_t *valid_bits,
                                     uint8_t rx_align, bool check_CRC)
{
    // Prepare values for BitFramingReg
    uint8_t txLastBits = valid_bits ? *valid_bits : 0;
    uint8_t bitFraming = (rx_align << 4) + txLastBits; // rx_align = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    write_PCD_command(PCD_Idle);                             // Stop any active command.
    write_register(ComIrqReg, &(uint8_t){0x7F});             // Clear all seven interrupt request bits
    write_register(FIFOLevelReg, &(uint8_t){0x80});          // FlushBuffer = 1, FIFO initialization
    write_register_stream(FIFODataReg, send_len, send_data); // Write send_data to the FIFO
    write_register(BitFramingReg, &bitFraming);              // Bit adjustments
    write_PCD_command(command);                              // Execute the command
    if (command == PCD_Transceive)
    {
        PCD_set_register_bit_mask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete
    uint8_t n;
    uint8_t irqVal;
    uint8_t status1;
    uint8_t status2;
    uint16_t i;

    for (i = 2000; i > 0; i--)
    {
        read_data_byte(ComIrqReg, &irqVal); // ComIrqReg[7..0]
        if (irqVal & wait_irq)
        {
            break;
        }
        if (irqVal & 0x01)
        { // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }

    // Check for timeout
    if (i == 0)
    {
        return STATUS_TIMEOUT;
    }

    // Error handling
    uint8_t error;
    read_data_byte(ErrorReg, &error); // ErrorReg[7..0]
    if (error & 0x13)
    { // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    // Handle back data if any
    if (back_data && back_len)
    {
        read_data_byte(FIFOLevelReg, &n); // Number of bytes in the FIFO
        if (n > *back_len)
        {
            return STATUS_NO_ROOM;
        }
        *back_len = n;                               // Store number of bytes returned
        read_data_stream(FIFODataReg, back_data, n); // Get received data from FIFO
    }

    if (valid_bits)
    {
        read_data_byte(ControlReg, &n);
        *valid_bits = n & 0x07;
    }

    // Handle collision errors
    if (error & 0x08)
    { // CollErr
        return STATUS_COLLISION;
    }

    // Return status
    return STATUS_OK;
}

StatusCode PCD_transceive_data(uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint8_t *back_len,
                               uint8_t *valid_bits, uint8_t rx_align, bool check_CRC)
{
    return PCD_communicate_with_picc(PCD_Transceive, 0x30, send_data, send_len, back_data, back_len, valid_bits, rx_align, check_CRC);
}

StatusCode PICC_REQA_or_WUPA(PICC_Command command, uint8_t *buffer_ATQA, uint8_t *buffer_size)
{
    uint8_t valid_bits;
    uint8_t wait_irq;
    uint8_t back_data[18]; // ATQA can be 2 bytes long
    uint8_t back_len;

    // Only used in PICC_HaltA, otherwise unused
    if (command == PICC_CMD_HLTA)
    {
        wait_irq = 0x10;
        valid_bits = 0;
    }
    else
    {
        wait_irq = 0x30;
        valid_bits = 7; // According to ISO/IEC 14443-3 7.2.1
    }

    // Send the command
    StatusCode result = PCD_transceive_data(&command, 1, back_data, &back_len, &valid_bits, 0, false);
    if (result == STATUS_OK)
    {
        // ATQA must be exactly 2 bytes. Anything else must be interpreted as error.
        if (back_len != 2 || valid_bits != 0)
        {
            result = STATUS_ERROR;
        }
    }

    // Write ATQA response
    if (buffer_ATQA && buffer_size)
    {
        if (*buffer_size < 2)
        {
            return STATUS_NO_ROOM;
        }
        *buffer_size = 2;
        buffer_ATQA[0] = back_data[0];
        buffer_ATQA[1] = back_data[1];
    }

    return result;
}

StatusCode PICC_request_A(uint8_t *buffer_ATQA, uint8_t *buffer_size)
{
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, buffer_ATQA, buffer_size);
}

bool PICC_new_card_present()
{
    uint8_t buffer_ATQA[2];
    uint8_t buffer_size = sizeof(buffer_ATQA);
    StatusCode result = PICC_request_A(buffer_ATQA, &buffer_size);

    return (result == STATUS_OK || result == STATUS_COLLISION);
}

StatusCode PCD_calculate_CRC(uint8_t *data, uint32_t length, uint8_t *result)
{
    write_PCD_command(PCD_Idle);                      // Stop any active command.
    write_register(DivIrqReg, &(uint8_t){0x04});      // Clear the CRCIRq interrupt request bit
    write_register(FIFOLevelReg, &(uint8_t){0x80});   // FlushBuffer = 1, FIFO initialization
    write_register_stream(FIFODataReg, length, data); // Write data to the FIFO
    write_PCD_command(PCD_CalcCRC);                   // Start the calculation

    // For timeout
    write_register(ComIrqReg, &(uint8_t){0x7F});   // Clear all seven interrupt request bits
    write_register(TReloadRegH, &(uint8_t){0x0F}); // Reload timer with 0xFA0 = 4000, ie 100ms before timeout.
    write_register(TReloadRegL, &(uint8_t){0xA0});
    // TODO: clear arr interrupts. configure timer for 90ms.

    // Wait for the CRC calculation to complete. Check for the register to
    // indicate that the CRC calculation is complete in a loop. If the
    // calculation is not indicated as complete in ~100ms, then time out
    // the operation.

    uint8_t irqVal;
    do
    {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n;
        read_data_byte(DivIrqReg, &n);
        if (n & 0x04)
        {                                // CRCIRq bit set - calculation done
            write_PCD_command(PCD_Idle); // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            read_data_byte(CRCResultRegL, &result[0]);
            read_data_byte(CRCResultRegH, &result[1]);

            // Reset back to starting timer
            write_register(TReloadRegH, &(uint8_t){0x03}); // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
            write_register(TReloadRegL, &(uint8_t){0xE8});

            return STATUS_OK;
        }
        read_data_byte(ComIrqReg, &irqVal); // ComIrqReg[7..0]

    } while (!(irqVal & 0x01));
    // 100ms passed and nothing happened. Communication with the MFRC522 might be down.

    // Reset back to starting timer
    write_register(TReloadRegH, &(uint8_t){0x03}); // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    write_register(TReloadRegL, &(uint8_t){0xE8});

    return STATUS_TIMEOUT;
}

StatusCode PICC_select(Uid *uid, uint8_t valid_bits)
{
    bool uidComplete;
    bool selectDone;
    bool useCascadeTag;
    uint8_t cascadeLevel = 1;
    StatusCode result;
    uint8_t count;
    uint8_t checkBit;
    uint8_t index;
    uint8_t uidIndex;             // The first index in uid->uidByte[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];            // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    uint8_t bufferUsed;           // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    uint8_t rxAlign;              // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;           // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
    uint8_t *responseBuffer;
    uint8_t responseLength;

    PCD_clear_register_bit_mask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while (!uidComplete)
    {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascadeLevel)
        {
        case 1:
            buffer[0] = PICC_CMD_SEL_CL1;
            uidIndex = 0;
            useCascadeTag = valid_bits && uid->size > 4; // When we know that the UID has more than 4 bytes
            break;

        case 2:
            buffer[0] = PICC_CMD_SEL_CL2;
            uidIndex = 3;
            useCascadeTag = valid_bits && uid->size > 7; // When we know that the UID has more than 7 bytes
            break;

        case 3:
            buffer[0] = PICC_CMD_SEL_CL3;
            uidIndex = 6;
            useCascadeTag = false; // Never used in CL3.
            break;

        default:
            return STATUS_INTERNAL_ERROR;
            break;
        }

        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = valid_bits - (8 * uidIndex);
        if (currentLevelKnownBits < 0)
        {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag)
        {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
        if (bytesToCopy)
        {
            uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytesToCopy > maxBytes)
            {
                bytesToCopy = maxBytes;
            }
            for (count = 0; count < bytesToCopy; count++)
            {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag)
        {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = false;
        while (!selectDone)
        {
            // Find out how many bits and bytes to send and receive.
            if (currentLevelKnownBits >= 32)
            { // All UID bits in this Cascade Level are known. This is a SELECT.
                // Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // TODO: check
                //  Calculate CRC_A
                result = PCD_calculate_CRC(buffer, 7, &buffer[7]);
                if (result != STATUS_OK)
                {
                    return result;
                }
                txLastBits = 0; // 0 => All 8 bits are valid.
                bufferUsed = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer = &buffer[6];
                responseLength = 3;
            }
            else
            { // This is an ANTICOLLISION.
                // Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                txLastBits = currentLevelKnownBits % 8;
                count = currentLevelKnownBits / 8;     // Number of whole bytes in the UID part.
                index = 2 + count;                     // Number of whole bytes: SEL + NVB + UIDs
                buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
                bufferUsed = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer = &buffer[index];
                responseLength = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                       // Having a separate variable is overkill. But it makes the next line easier to read.
            write_register(BitFramingReg, &(uint8_t){(rxAlign << 4) + txLastBits}); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = PCD_transceive_data(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
            if (result == STATUS_COLLISION)
            { // More than one PICC in the field => collision.
                uint8_t valueOfCollReg;
                read_data_byte(CollReg, &valueOfCollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20)
                {                            // CollPosNotValid
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0)
                {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits)
                { // No progress - should not happen
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits = collisionPos;
                count = currentLevelKnownBits % 8; // The bit to modify
                checkBit = (currentLevelKnownBits - 1) % 8;
                index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index] |= (1 << checkBit);
            }
            else if (result != STATUS_OK)
            {
                return result;
            }
            else
            { // STATUS_OK
                if (currentLevelKnownBits >= 32)
                {                      // This was a SELECT.
                    selectDone = true; // No more anticollision
                                       // We continue below outside the while.
                }
                else
                { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while (!selectDone)

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++)
        {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0)
        { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = PCD_calculate_CRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK)
        {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
        {
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04)
        { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else
        {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    } // End of while (!uidComplete)

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;

    return STATUS_OK;
}

bool PICC_read_card_serial(Uid *current_uid)
{
    StatusCode result = PICC_select(current_uid, 0);
    return (result == STATUS_OK);
}
