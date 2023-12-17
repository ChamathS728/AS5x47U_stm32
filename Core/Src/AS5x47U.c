

#include "AS5x47U.h"


/* Initialisation Functions */


/* Data Acquistion Functions */
HAL_StatusTypeDef AS5x47U_readPosition(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_readVelocity(AS5x47U* enc_ptr); 

/* Low Level Functions */
// NOTE - SPI commands here work with 24bit frames for CRC 8bit checks + we don't need the speed of 16bit frames
HAL_StatusTypeDef AS5x47U_readRegister(AS5x47U* enc_ptr, uint16 reg_addr, int16_t output) {
    // NOTE: Consider adding logic related to the ERR bits: WARNING and ERROR respectively
/*
    Inputs:
        enc_ptr: Pointer to the AS5x47U struct instance that stores all information
        reg_addr: 14 bit address indicating which register to be read over SPI -> stored as uint16_t

    24 bit (3 byte) SPI transaction here
    
    txBuffer Structure
        Bit23 = 0
        Bit22 = 1 for reading
        Bits 21:8 to store the register address (14 bits)
        Bits 7:0 has the CRC that we send across


*/
    // Set up transmission buffer
    uint8_t txBuff[3]; // 24 bit transactions = 3 * 8 = 3 bytes

    // Set the 15th bit (bit 14) to 1 for read; the 16th (bit 15) bit is already 0
    uint16_t tempUpper = reg_addr | (1 << 14); 
    /*
        tempUpper:
            Bit 15 is 0 since it isn't touched
            Bit 14 is 1 since we want to read
            Bits 13 to 0 contain the address

        crc:
            Bits 7 to 0 here are all part of the crc
    */
    txBuff[0] = (uint8_t) (tempUpper >> 8);  // shift down by 8 bits to get the upper 8 bits
    txBuff[1] = (uint8_t) (tempUpper % 256); // Modulo by 2^8 to get the lower 8 bits 
    txBuff[2] = enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value
    
    // Transmit over SPI and store away into a 24bit buffer
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(enc_ptr->hspi, txBuff, enc_ptr->rxBuffer, 1, HAL_MAX_DELAY);
    
    // Check that the CRC provided matches what we stored (enc_ptr->last_crc)
    uint8_t rxTop = enc_ptr->rxBuffer[0];
    uint8_t rxMid = enc_ptr->rxBuffer[1];
    uint8_t rxCRC = enc_ptr->rxBuffer[2];
    if (rxCRC != enc_ptr->last_crc) {
        return HAL_ERROR;
    }

    // Use bits 21:8 as the data bits that we actually want to read (14 bit number)
    rxTop = rxTop % 64; // Modulo by 2^6 to get the lower 6 bits
    
    // Stitch the top and middle entries together into an int16_t (which only has 14 notable bits)
    output = (rxTop << 8) | rxMid;

    

    return result;
}



HAL_StatusTypeDef AS5x47U_readRegisters(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_writeRegister(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_calcCRC(AS5x47U* enc_ptr, uint16_t data) {
    /*
    Rationale
        - Make a copy of the data to calculate CRC from
        - Initialise the divisor (11101)
        - Initialise a bitshift number as 12 (we bit shift things left or right by 12 initially)

        - Set up for loop to go from 12 to 1 I think
        - Bit shift it right so that we only get the 5 most significant bits
        - XOR this value with the divisor of 11101 = 29 (store it as an 8 bit number I guess)
        - Bit shift the result left until it aligns with the original setup (same number as the right shift)
            - Now we need to replace those 5 bits of the data with the result
        - 1 1101 0000 0000 0000
    */


    // Polynomial to use is x^4 + x^3 + x^2 + 1 -> 11101 = 29
    int32_t pad = 12; // We pad the divisor by 12 bits to get it to 17 bits
    int32_t poly = 29; // 0000 0000 0000 0000 0000 0000 0001 1101 or 11101


    // int32_t divisor  = 29 << 12; // same as 11101 in decimal, but shifted 12 bits up so that it is 17 bit
    // We'll do this CRC calculation on bits 23:8 -> 16 bit input
    // CRC should be 8 bits

    // Implementation based on wikipedia lol
    
    // Pad data by 3 bits so that it is 17 bits temporarily
    uint32_t blah = data << 3;

    // Calculate the divisor
    uint32_t divisor = (uint32_t) data &  1;    

    // Sequentially "divide" data by XOR'ing with the divisor (^ operand)
    for (int i = 0; i < pad; i++) {
        // Divide
        blah = blah ^ divisor;

        // Recalculate the divisor
    }

} // Calculation based on bits 23:8 -> Page 27 of 61: CRC Checksum
