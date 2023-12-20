

#include "AS5x47U.h"


/* Initialisation Functions */
HAL_StatusTypeDef AS5x47U_init(AS5x47U* enc_ptr, SPI_HandleTypeDef* hspi, GPIO_TypeDef* enc_CS_port, uint16_t, enc_CS_pin) {

    // SPI initialisation
    enc_ptr->hspi = hspi;
    enc_ptr->CS_port = enc_CS_port;
    enc_ptr->CS_pin = enc_CS_pin;

    // Configuration information
    enc_ptr->rxBuffer = {0,0,0};    // NOTE - 3 bytes in length for 24 bit transactions specifically

    // Actual data stored away
    enc_ptr->velocity = 0;
    enc_ptr->angle_comp = 0;
    enc_ptr->CORDIC_mag = 0;

    // Calibration information
    
    // Crc value
    enc_ptr->last_crc = 196; // 0xC4; initial value for the CRC
    enc_ptr->crcPoly = 29;   // 0x1D; CRC polynomial = 0x1D = 11101   


    return HAL_OK;
}

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

    // Make temporary variable containing the message for CRC calculation
    uint16_t temp = (txBuff[0] << 8) | txBuff[1];
    HAL_StatusTypeDef result = AS5x47U_calcCRC(enc_ptr, temp);

    txBuff[2] = enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value
    
    // Transmit over SPI and store away into a 24bit buffer
    // NOTE - .ioc file should specify SPI data length as 24 bits
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive(enc_ptr->hspi, txBuff, enc_ptr->rxBuffer, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);
    
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


// HAL_StatusTypeDef AS5x47U_readRegisters(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_writeRegister(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t input) {
    /*
    Inputs:
        enc_ptr: Pointer to the AS5x47U struct instance that stores all information
        reg_addr: 14 bit address indicating which register to be read over SPI -> stored as uint16_t
        input: Data to be written to the specified register
    */

    return HAL_OK;
}

HAL_StatusTypeDef AS5x47U_calcCRC(AS5x47U* enc_ptr, uint16_t crcData) {
    /*
        Inputs:
            enc_ptr: pointer to AS5x47U instance
            crcData: Data to calculate a CRC value for
        Output:
            HAL_STATUS detailing how the calculation went

        Pseudocode (inspired from here: https://www.educative.io/answers/how-to-calculate-the-crc)
        1. Get the length of the divisor polynomial (N)
        2. Append N-1 bits of 0 to the data packet
        3. Perform modulo-2 division (crcData appended divided by the divisor)
        4. Store the remainder in enc_ptr (should be an 8 bit number to be appended in the message)

        NOTE: Modulo-2 division could maybe be optimised
    */
    // 1. Get any necessary variable lengths (binary lengths)
    int N = 5; // Hardcoded value as the divisor polynomial is fixed at 11101
    int dataLen = 16; // crcData is always 16 bits long for 24bit transfers

    // 2. 
    uint32_t crcData_appended = crcData << (N - 1); // Could bitshift left by 4 as well directly to save a variable

    // 3. Divide a 20 bit number by a 5 bit number via XOR's -> need 15 rounds of division at most
    for (int i = (dataLen - 1); i <= 0; i--) { // i starts from 15 and goes down to 0 to give 16 iterations
        // Bitshift left the divisor by i bits for division
        uint32_t divisor = (enc_ptr->crcPoly << i);

        // Check if we need to XOR with 0 due to divisor and crcData_appended not being aligned at this stage
        if (crcData_appended % (1 << (i + N)) != crcData_appended) {
            // Aligned data, so XOR with divisor of the proper size
            crcData_appended = crcData_appended ^ divisor;
        }
        else {
            // Not aligned, so XOR with 0 to maintain data size; later iteration should have a divisor with the correct size
            crcData_appended = crcData_appended ^ 0;            
        }
    }

    // 4.
    enc_ptr->last_crc = crcData_appended;

    return HAL_OK;
}

HAL_StatusTypeDef AS5x47U_verifyCRC(AS5x47U* enc_ptr, uint16_t receivedData) {

    return HAL_OK;
}