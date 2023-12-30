

#include "AS5x47U.h"


/* Initialisation Functions */
HAL_StatusTypeDef AS5x47U_init(AS5x47U* enc_ptr, SPI_HandleTypeDef* hspi, GPIO_TypeDef* enc_CS_port, uint16_t enc_CS_pin, CRC_HandleTypeDef* hcrc) {

    // SPI initialisation
    enc_ptr->hspi = hspi;
    enc_ptr->CS_port = enc_CS_port;
    enc_ptr->CS_pin = enc_CS_pin;

    // Configuration information
    // enc_ptr->rxBuffer[3] = {0,0,0};    // NOTE - 3 bytes in length for 24 bit transactions specifically
    enc_ptr->rxBuffer16 = 0;

    // Actual data stored away
    enc_ptr->velocity = 0;
    enc_ptr->angle_comp = 0.21;
    enc_ptr->angle_uncomp = 0;
    enc_ptr->CORDIC_mag = 0;

    // Calibration information
    
    // Crc value
    enc_ptr->last_crc = 0; // 0xC4; initial value for the CRC
    enc_ptr->crcPoly = 29;   // 0x1D; CRC polynomial = 0x1D = 11101   


    // Error register
    enc_ptr->err_reg = 0;
    enc_ptr->warningBit = 0;
    enc_ptr->errorBit = 0;
    enc_ptr->hcrc = hcrc;

    // Create the CRC table
    AS5x47U_calcCRCTable(enc_ptr);

    return HAL_OK;
}

/* Data Acquistion Functions */
HAL_StatusTypeDef AS5x47U_readPositionDAE(AS5x47U* enc_ptr) {
    // Initialise variables
    uint16_t posRaw;

    // Read the angle compensated register
    // HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, ANGLECOM, &posRaw);
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, ANGLECOM, &posRaw);

    if (enc_ptr->warningBit || enc_ptr->errorBit) {
    	// Either warning or error bit set; check the error register
    	AS5x47U_readERRFL(enc_ptr);
    }

    if (result != HAL_OK) {
        return HAL_ERROR;
    }

    // Convert posRaw into int16_t -> casting should work
    int16_t temp = (int16_t) posRaw;
    enc_ptr->angle_comp = (float) temp/16384. * 360.;


    return result;
}

HAL_StatusTypeDef AS5x47U_readPositionNoDAE(AS5x47U* enc_ptr) {
    // Initialise variables
    uint16_t posRaw;

    // Read the angle uncompensated register
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, ANGLEUNC, &posRaw);

    // Convert posRaw into int16_t -> casting should work
    int16_t temp = (int16_t) posRaw;
    enc_ptr->angle_uncomp = (float) temp/16384.0 * 360.;

    return result;
}

HAL_StatusTypeDef AS5x47U_readVelocity(AS5x47U* enc_ptr) {
    // Initialise variables
    uint16_t velRaw;

    // Read the velocity register
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, VEL, &velRaw);

    // Convert velRaw into int16_t -> casting should work
    int16_t temp = (int16_t) velRaw;
    enc_ptr->velocity = (float) temp/16384.0 * 360.;


    return result;    
}

HAL_StatusTypeDef AS5x47U_readERRFL(AS5x47U* enc_ptr) {
    // Initialise variables
    uint16_t temp;

    // Read error flag register
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, ERRFL, &temp);

    // Get the 8 bits ( [16:8] ) that matter
    uint8_t error_flag = temp >> 8;

    // Store them in the enc_ptr instance
    enc_ptr->err_reg = error_flag;

    // Reset the error and warning bits
    enc_ptr->warningBit = 0;
	enc_ptr->errorBit = 0;

    return result;
}


/* Low Level Functions */
// NOTE - SPI commands here work with 24bit frames for CRC 8bit checks + we don't need the speed of 16bit frames
HAL_StatusTypeDef AS5x47U_readRegister(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t* output) {
/*
NOTE - Consider adding logic related to the ERR bits: WARNING and ERROR respectively

    Inputs:
        enc_ptr: Pointer to the AS5x47U struct instance that stores all information
        reg_addr: 14 bit address indicating which register to be read over SPI -> stored as uint16_t

    24 bit (3 byte) SPI transaction here
    
    txBuffer Structure
        Bit23 = 0
        Bit22 = 1 for readin
        Bits 21:8 to store the register address (14 bits)
        Bits 7:0 has the CRC that we send across


*/
    HAL_StatusTypeDef result = HAL_OK;


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

    uint8_t tempBuff[2] = {txBuff[0], txBuff[1]};



    // Make temporary variable containing the message for CRC calculation
    enc_ptr->last_crc = HAL_CRC_Accumulate(enc_ptr->hcrc, (uint32_t*) tempBuff, sizeof(tempBuff)/sizeof(uint8_t));
    // enc_ptr->last_crc = HAL_CRC_Calculate(enc_ptr->hcrc, (uint32_t*) tempBuff, sizeof(tempBuff)/sizeof(uint8_t));
    // HAL_StatusTypeDef result = AS5x47U_calcCRC(enc_ptr, tempUpper);
    // AS5x47U_calcCRC_new(enc_ptr, tempBuff, 2);
    
    txBuff[2] = enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value
    
    // Transmit over SPI and store away into a 24bit buffer
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(enc_ptr->hspi, txBuff, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);

    // Send nothing to get the actual result we want
    result = _AS5x47U_readRegisterNOP(enc_ptr);
    
    uint8_t rxTop = enc_ptr->rxBuffer[0];
    uint8_t rxMid = enc_ptr->rxBuffer[1];
    uint8_t rxCRC = enc_ptr->rxBuffer[2];

    // Check the warning and error bits
    int warningBit = rxTop & 0x80;
    int errorBit = rxTop & 0x40;

    if (warningBit) {
        result = HAL_ERROR;
        enc_ptr->warningBit = 1;
    }
    if (errorBit) {
    	result = HAL_ERROR;
    	enc_ptr->errorBit = 1;
    }

    // Check that the CRC provided matches the calculated CRC based on this data
    
    tempBuff[0] = rxTop;
    tempBuff[1] = rxMid;

    enc_ptr->last_crc = HAL_CRC_Accumulate(enc_ptr->hcrc, (uint32_t*) tempBuff, sizeof(tempBuff)/sizeof(uint8_t));
    // enc_ptr->last_crc = HAL_CRC_Calculate(enc_ptr->hcrc, (uint32_t*) tempBuff, sizeof(tempBuff)/sizeof(uint8_t));
    // AS5x47U_calcCRC_new(enc_ptr, tempBuff, 2);

    if ((uint32_t) rxCRC != enc_ptr->last_crc) {
        result = HAL_ERROR;
    }

    // Use bits 21:8 as the data bits that we actually want to read (14 bit number)
    rxTop = rxTop % 64; // Modulo by 2^6 to get the lower 6 bits
    
    // Stitch the top and middle entries together into an uint16_t (which only has 14 notable bits)
    *output = (rxTop << 8) | rxMid;

    return result;
}

HAL_StatusTypeDef _AS5x47U_readRegisterNOP(AS5x47U* enc_ptr) {
    /*
        Only use this function within API calls

        @brief Reads the NOP register
        @param Pointer to the AS5x47U instance being used
        @return HAL_STATUS based on whether the SPI transaction was successful
    */
   
    // Prepare txBuffer containing the NOP register and a 1 for the read
    uint8_t txBuff[3];

    // Set the 15th bit (bit 14) to 1 for read; the 16th (bit 15) bit is already 0
    uint16_t tempUpper = NOP | (1 << 14); 

    txBuff[0] = (uint8_t) (tempUpper >> 8);  // shift down by 8 bits to get the upper 8 bits
    txBuff[1] = (uint8_t) (tempUpper % 256); // Modulo by 2^8 to get the lower 8 bits 

    uint8_t tempBuff[2] = {txBuff[0], txBuff[1]};

    // Make temporary variable containing the message for CRC calculation
    enc_ptr->last_crc = HAL_CRC_Accumulate(enc_ptr->hcrc, (uint32_t*) tempBuff, sizeof(tempBuff)/sizeof(uint8_t));
    // enc_ptr->last_crc = HAL_CRC_Calculate(enc_ptr->hcrc, (uint32_t*) tempBuff, sizeof(tempBuff)/sizeof(uint8_t));

    txBuff[2] = (uint8_t) enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value   

    // Transmit and receive to/from the NOP register -> rxBuffer now has what we want
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(enc_ptr->hspi, txBuff, enc_ptr->rxBuffer, sizeof(txBuff)/sizeof(uint8_t), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);

    return result;
}

HAL_StatusTypeDef _AS5x47U_readRegisterNOP16(AS5x47U* enc_ptr) {
    // Prepare txBuffer containing the NOP register and a 1 for the read
    uint16_t txBuff = NOP | (1 << 14); ;

    // Transmit and receive to/from the NOP register -> rxBuffer now has what we want
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(enc_ptr->hspi, 
                                                       (uint8_t*) &txBuff,
                                                       (uint8_t*) &(enc_ptr->rxBuffer16),
                                                       2,
                                                       HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);

    return result;    
}

HAL_StatusTypeDef AS5x47U_readRegister16(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t* output) {

    // Set up transmission buffer
    // Set the 15th bit (bit 14) to 1 for read; the 16th (bit 15) bit is already 0
    uint16_t txBuff = reg_addr | (1 << 14); ; // 24 bit transactions = 3 * 8 = 3 bytes
    /*
        txBuff:
            Bit 15 is 0 since it isn't touched
            Bit 14 is 1 since we want to read
            Bits 13 to 0 contain the address
    */
    // Transmit over SPI and store away into a 24bit buffer
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(enc_ptr->hspi, 
                                                (uint8_t* )&txBuff,
                                                2,
                                                HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);

    // Send nothing to get the actual result we want
    result = _AS5x47U_readRegisterNOP16(enc_ptr);
    
    uint8_t rxTop = enc_ptr->rxBuffer16 >> 8;
    uint8_t rxBot = enc_ptr->rxBuffer16 % 256;

    // Check the warning and error bits
    int warningBit = rxTop & 0x80;
    int errorBit = rxTop & 0x40;

    if (warningBit) {
        result = HAL_ERROR;
        enc_ptr->warningBit = 1;
    }
    if (errorBit) {
    	result = HAL_ERROR;
    	enc_ptr->errorBit = 1;
    }

    // Use bits 21:8 as the data bits that we actually want to read (14 bit number)
    rxTop = rxTop % 64; // Modulo by 2^6 to get the lower 6 bits
    
    // Stitch the top and middle entries together into an uint16_t (which only has 14 notable bits)
    *output = (rxTop << 8) | rxBot;

    return result;
}

HAL_StatusTypeDef AS5x47U_writeRegister(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t input) {
    /*
    Inputs:
        enc_ptr: Pointer to the AS5x47U struct instance that stores all information
        reg_addr: 14 bit address indicating which register to be read over SPI -> stored as uint16_t
        input: 14 bit input value to be written to the specified register

    Register writes happen in two steps
        1. Send packet containing the address of the register to write to
            NOTE: MOSI line has this packet; MISO won't have anything
        2. Send packet containing the data we want to write to that register
            NOTE: MOSI line has this packet, MISO line has a packet containing the old contents of the register
            Current implementation ignores the old contents 
    */

    // Send first packet with target address
    HAL_StatusTypeDef result = AS5x47U_calcCRC(enc_ptr, reg_addr); // CRC calc on the "data"

    uint8_t txBuff[3]; // Transmission buffer
    txBuff[0] = (uint8_t) (reg_addr >> 8);  // shift down by 8 bits to get the upper 8 bits
    txBuff[1] = (uint8_t) (reg_addr % 256); // Modulo by 2^8 to get the lower 8 bits 
    txBuff[2] = enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value

    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(enc_ptr->hspi, txBuff, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);    

    // Send second packet with data to be written
    result = AS5x47U_calcCRC(enc_ptr, input); // CRC calc on the actual data to be written
    txBuff[0] = (uint8_t) (input >> 8);  // shift down by 8 bits to get the upper 8 bits
    txBuff[1] = (uint8_t) (input % 256); // Modulo by 2^8 to get the lower 8 bits 
    txBuff[2] = enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value    

    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(enc_ptr->hspi, txBuff, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);    
    
    return result;
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
    uint32_t crcData_appended = crcData << (N-1); // Could bitshift left by 4 as well directly to save a variable

    // 3. Divide a 20 bit number by a 5 bit number via XOR's -> need 15 rounds of division at most
    // for (int i = (dataLen - 1); i >= 0; i--) { // i starts from 15 and goes down to 0 to give 16 iterations
    //     // Bitshift left the divisor by i bits for division
    //     uint32_t divisor = (enc_ptr->crcPoly << i);

    //     // Check if we need to XOR with 0 due to divisor and crcData_appended not being aligned at this stage
    //     if (crcData_appended % (1 << (i + N)) != crcData_appended) {
    //         // Aligned data, so XOR with divisor of the proper size
    //         crcData_appended = crcData_appended ^ divisor;
    //     }
    //     else {
    //         // Not aligned, so XOR with 0 to maintain data size; later iteration should have a divisor with the correct size
    //         crcData_appended = crcData_appended ^ 0;  

    //         //           
    //     }
    // }

    int i = dataLen - 1;
    while (i >= 0) {
        // Bitshift left the divisor by i bits for division
        uint32_t divisor = (enc_ptr->crcPoly << i);

        int rise = i + N - 1;
        int div = 1 << rise;
        uint32_t temp = crcData_appended & div;

        // Check if we need to XOR with 0 due to divisor and crcData_appended not being aligned at this stage
        if (temp) {
            // Aligned data, so XOR with divisor of the proper size
            crcData_appended = crcData_appended ^ divisor;
        }
        else {
            // Not aligned, so XOR with 0 to maintain data size; later iteration should have a divisor with the correct size
            crcData_appended = crcData_appended ^ 0;  
        }        

        // Decrement the counter
        i--;

    }

    // 4.
    enc_ptr->last_crc = crcData_appended;

    return HAL_OK;
}

void AS5x47U_calcCRC_new(AS5x47U* enc_ptr, uint8_t* bytes, int numOfBytes) {
    uint8_t crc = 0;

    for (int i = 0; i < numOfBytes; i++) {
        uint8_t data = bytes[i] ^ crc;

        crc = (uint8_t) (enc_ptr->crcTable[data]);
    }

    enc_ptr->last_crc = crc;
}

void AS5x47U_calcCRCTable(AS5x47U* enc_ptr) {
    const uint8_t generator = enc_ptr->crcPoly;
    /* iterate over all byte values 0 - 255 */
    for (int dividend = 0; dividend < 256; dividend++)
    {
        uint8_t currByte = (uint8_t) dividend;
        /* calculate the CRC-8 value for current byte */
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((currByte & 0x80) != 0)
            {
                currByte <<= 1;
                currByte ^= generator;
            }
            else
            {
                currByte <<= 1;
            }
        }
        /* store CRC value in lookup table */
        enc_ptr->crcTable[dividend] = currByte;
    }
}

void CRC_calc_HAL(AS5x47U* enc_ptr, uint8_t* bytes, size_t numOfBytes) {
    /*
    Following the steps from the CRC application note from ST: AN4187

    1. Enable CRC peripheral clock via the RCC peripheral
    2. Set the initial CRC value via the CRC_INIT
    3. Set the I/O reverse bit order through the REV_IN[1:0] and REV_OUT[1:0] bits in the CRC_CR register
    4. Set the polynomial size and coefficients through the POLYSIZE[1:0] bits in the CRC_CR and CRC_POL registers respectively
    5. Reset the CRC peripheral through the reset bit of the CRC_CR register
    6. Set the data to the CRC data register
    7. Read the contents of the CRC data register
    8. Disable the CRC peripheral clock
    */
}


