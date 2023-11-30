

#include "AS5x47U.h"


/* Initialisation Functions */


/* Data Acquistion Functions */
HAL_StatusTypeDef AS5x47U_readPosition(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_readVelocity(AS5x47U* enc_ptr); 

/* Low Level Functions */
// NOTE - SPI commands here work with 24bit frames for CRC 8bit checks + we don't need the speed of 16bit frames
HAL_StatusTypeDef AS5x47U_readRegister(AS5x47U* enc_ptr, uint16 reg_addr) {
    // 24 bit transaction SPI TransmitReceive

    // Set up transmission buffer
    int16_t txBuff[3]; // 24 bit transactions = 3 * 8 = 3 bytes
    /*
        txBuffer Structure
        Bit23 = 0
        Bit22 = 1 for reading
        Bits 21:8 to store the register address (14 bits)
        Bits 7:0 has the CRC that we send across
    */
    uint16_t tempUpper = reg_addr | (1 << 14); // Set the 14th bit to 1 for read; the 15th bit is already 0
    uint8_t tempLower = 2; //FIXME - CRC calculation should be done for this value

    


    return HAL_OK;
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
