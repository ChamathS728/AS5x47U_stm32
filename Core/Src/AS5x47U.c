

#include "AS5x47U.h"


/* Initialisation Functions */
AS5x47U* AS5x47U_init(SPI_HandleTypeDef* hspi, 
                      GPIO_TypeDef* enc_CS_port, 
                      uint16_t enc_CS_pin) {

    // Create handle
    AS5x47U* enc_ptr = NULL;

    // SPI initialisation
    enc_ptr->hspi = hspi;
    enc_ptr->CS_port = enc_CS_port;
    enc_ptr->CS_pin = enc_CS_pin;

    // Configuration information
    enc_ptr->rxBuffer[0] = 0;    // NOTE - 3 bytes in length for 24 bit transactions specifically
    enc_ptr->rxBuffer[1] = 0;

    // Actual data stored away
    enc_ptr->velocity = 0;
    enc_ptr->angle_comp = 0;
    enc_ptr->angle_uncomp = 0;
    enc_ptr->CORDIC_mag = 0;

    // Calibration information
    

    return enc_ptr;
}

/* Data Acquistion Functions */
HAL_StatusTypeDef AS5x47U_readPositionDAE(AS5x47U* enc_ptr) {
    // Initialise variables
    int16_t posRaw;

    // Read the angle compensated register
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, ANGLECOM, &posRaw);

    // Convert velRaw into actual value
    enc_ptr->angle_comp = (float) posRaw/16384. * 360.;

    return result;
}

HAL_StatusTypeDef AS5x47U_readPositionNoDAE(AS5x47U* enc_ptr) {
    // Initialise variables
    int16_t posRaw;

    // Read the angle uncompensated register
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, ANGLEUNC, &posRaw);

    // Convert velRaw into actual value
    enc_ptr->angle_comp = (float) posRaw/16384. * 360.;

    return result;
}

HAL_StatusTypeDef AS5x47U_readVelocity(AS5x47U* enc_ptr) {
    // Initialise variables
    int16_t velRaw;

    // Read the velocity register
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, VEL, &velRaw);

    // Convert velRaw into actual value
    enc_ptr->angle_comp = (float) velRaw/16384. * 360.;

    return result;    
}

HAL_StatusTypeDef AS5x47U_readERRFL(AS5x47U* enc_ptr) {
    /*
        11 bits go like this

        bit 10  : CORDIC overflow
        bit 9   : OffCompNotFinished
        bit 8   : BRKHALL -> broken hall effect sensor
        bit 7   : WDTST -> Internal oscillator or watchdog isn't working correctly
        bit 6   : CRC error -> CRC error during SPI communication
        bit 5   : Command error -> SPI invalid command received
        bit 4   : Framing error -> Framing if SPI communication wrong
        bit 3   : P2ram error -> ECC has detected 2 uncorrectable errors in P2RAM in customer area
        bit 2   : P2ram warning -> ECC is correcting one bit of P2ram in customer area
        bit 1   : MagHalf -> 
        bit 0   : Agc-warning -> AGC value is 0LSB or 255LSB
    */

    int16_t* temp = NULL;
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, ERRFL, temp);
    uint16_t blah = (uint16_t) *temp;
    // Store the individual bits in an array
    for (int i = 0; i < 11; i++) {
      uint16_t sieve = (1 << i);
      enc_ptr->errReg[i] = (uint8_t) ((blah & sieve) >> i);    // Cast temp back to uint16_t
    } 

    return result;   
}

HAL_StatusTypeDef AS5x47U_readDIA(AS5x47U* enc_ptr) {
    /*
        13 bits go like this

        bit 12  : SPI_cnt -> SPI frame counter
        bit 11  : SPI_cnt -> SPI frame counter 
        bit 10  : Fusa_error -> Error flag broken Hall element
        bit 9   : AGC_finished -> Initial AGC settling finished
        bit 8   : Off comp finished -> Error flag offset compensation finished
        bit 7   : SinOff_fin -> Sine offset compensation finished
        bit 6   : CosOff_fin -> Cosine offset compensation finished
        bit 5   : MagHalf_flag -> Error flag magnitude is below half of target value
        bit 4   : Comp_h -> Warning flag AGC high
        bit 3   : Comp_l -> Warning flag AGC low
        bit 2   : Cordic_overflow -> Error flag CORDIC overflow
        bit 1   : LoopsFinished -> All magneto core loops finished
        bit 0   : Vdd_mode -> VDD supply mode:
    */

    int16_t* temp = NULL;
    HAL_StatusTypeDef result = AS5x47U_readRegister(enc_ptr, DIA, temp);
    uint16_t blah = (uint16_t) *temp;
    // Store the individual bits in an array
    for (int i = 0; i < 13; i++) {
      uint16_t sieve = (1 << i);
      enc_ptr->diaReg[i] = (uint8_t) ((blah & sieve) >> i);    // Cast temp back to uint16_t
    } 

    return result;   
}


/* Low Level Functions */
// NOTE - SPI commands here work with 24bit frames for CRC 8bit checks + we don't need the speed of 16bit frames
HAL_StatusTypeDef AS5x47U_readRegister(AS5x47U* enc_ptr, 
                                       uint16_t reg_addr, 
                                       int16_t* output) {
    
    // create txBuffer with address
    uint8_t txBuffer[2];
    reg_addr = reg_addr | 0x4000; // Set RW bit to 1 for read
    txBuffer[0] = (uint8_t) (reg_addr >> 8);
    txBuffer[1] = (uint8_t) (reg_addr % 256);
    
    if (enc_ptr->hspi->State != HAL_SPI_STATE_READY) {
    	return HAL_BUSY;
    }

    // Send 16-bit buffer with address 
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(enc_ptr->hspi,
                                                txBuffer,
												enc_ptr->rxBuffer,
                                                2, 
                                                HAL_MAX_DELAY);

    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);

    if (enc_ptr->hspi->State != HAL_SPI_STATE_READY) {
    	return HAL_BUSY;
    }

    // recreate txBuffer with NOP address
    uint16_t temp = NOP | 0x4000;
    txBuffer[0] = (uint8_t) (temp >> 8);
    txBuffer[1] = (uint8_t) (temp % 256);

    // Send 16-bit buffer with NOP address and receive simultaneously
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive(enc_ptr->hspi, 
                                     txBuffer, 
                                     enc_ptr->rxBuffer, 
                                     2, 
									 HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);

    if (enc_ptr->hspi->State != HAL_SPI_STATE_READY) {
    	return HAL_BUSY;
    }

    // Put the contents of the rxBuffer into the output
    uint8_t data = enc_ptr->rxBuffer[0] % 64;
    *output = (int16_t) (data << 8) | (enc_ptr->rxBuffer[1]);

    // Check the warning and error bits
    enc_ptr->warningBit = (enc_ptr->rxBuffer[0] & 0x80) >> 7;
    enc_ptr->errorBit = (enc_ptr->rxBuffer[0] & 0x40) >> 6;

    return result;
}

/*
HAL_StatusTypeDef AS5x47U_writeRegister(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t input) {

    // Inputs:
    //     enc_ptr: Pointer to the AS5x47U struct instance that stores all information
    //     reg_addr: 14 bit address indicating which register to be read over SPI -> stored as uint16_t
    //     input: 14 bit input value to be written to the specified register

    // Register writes happen in two steps
    //     1. Send packet containing the address of the register to write to
    //         NOTE: MOSI line has this packet; MISO won't have anything
    //     2. Send packet containing the data we want to write to that register
    //         NOTE: MOSI line has this packet, MISO line has a packet containing the old contents of the register
    //         Current implementation ignores the old contents 

    // Send first packet with target address
    HAL_StatusTypeDef result = AS5x47U_calcCRC(enc_ptr, reg_addr); // CRC calc on the "data"

    uint8_t txBuff[3]; // Transmission buffer
    txBuff[0] = (uint8_t) (reg_addr >> 8);  // shift down by 8 bits to get the upper 8 bits
    txBuff[1] = (uint8_t) (reg_addr % 256); // Modulo by 2^8 to get the lower 8 bits 
    txBuff[2] = enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value

    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(enc_ptr->hspi, txBuff, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);    

    // Send second packet with data to be written
    result = AS5x47U_calcCRC(enc_ptr, input); // CRC calc on the actual data to be written
    txBuff[0] = (uint8_t) (input >> 8);  // shift down by 8 bits to get the upper 8 bits
    txBuff[1] = (uint8_t) (input % 256); // Modulo by 2^8 to get the lower 8 bits 
    txBuff[2] = enc_ptr->last_crc; // Last 8 bits = 1 byte for the crc value    

    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(enc_ptr->hspi, txBuff, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc_ptr->CS_port, enc_ptr->CS_pin, GPIO_PIN_SET);    
    
    return result;
}
*/

void AS5x47U_printERRFL(AS5x47U* enc_ptr, UART_HandleTypeDef* huart) {
	float MSG[100] = {'\0'}; // NULL terminated string for printing
    AS5x47U_readERRFL(enc_ptr);
    sprintf(MSG, "ERRFL register:\r\nBit 10: %d\r\nBit 9: %d\r\nBit 8: %d\r\nBit 7: %d\r\nBit 6: %d\r\nBit 5: %d\r\nBit 4: %d\r\nBit 3: %d\r\nBit 2: %d\r\nBit 1: %d\r\nBit 0: %d\r\n\n",
            enc_ptr->errReg[10], enc_ptr->errReg[9], enc_ptr->errReg[8], enc_ptr->errReg[7], enc_ptr->errReg[6], enc_ptr->errReg[5], enc_ptr->errReg[4], enc_ptr->errReg[3], enc_ptr->errReg[2], enc_ptr->errReg[1], enc_ptr->errReg[0]);
    HAL_UART_Transmit(huart, (uint8_t*) MSG, sizeof(MSG), 100);
}

void AS5x47U_printDIA(AS5x47U* enc_ptr, UART_HandleTypeDef* huart) {
	float MSG[100] = {'\0'}; // NULL terminated string for printing
    AS5x47U_readDIA(enc_ptr);
    sprintf(MSG, "DIA register:\r\nBit 12: %d\r\nBit 11: %d\r\nBit 10: %d\r\nBit 9: %d\r\nBit 8: %d\r\nBit 7: %d\r\nBit 6: %d\r\nBit 5: %d\r\nBit 4: %d\r\nBit 3: %d\r\nBit 2: %d\r\nBit 1: %d\r\nBit 0: %d\r\n\n",
            enc_ptr->diaReg[12], enc_ptr->diaReg[11], enc_ptr->diaReg[10], enc_ptr->diaReg[9], enc_ptr->diaReg[8], enc_ptr->diaReg[7], enc_ptr->diaReg[6], enc_ptr->diaReg[5], enc_ptr->diaReg[4], enc_ptr->diaReg[3], enc_ptr->diaReg[2], enc_ptr->diaReg[1], enc_ptr->diaReg[0]);
    HAL_UART_Transmit(huart, (uint8_t*) MSG, sizeof(MSG), 100);    
}
