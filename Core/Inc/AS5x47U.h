/*
 * AS5x47U.h
 * AS5x47U Magnetic Encoder Driver over SPI
 *  Created on: Nov 10, 2023
 *      Author: Chamath Suraweera
 *
 */

#ifndef AS5X47U_SPI_DRIVER_H
#define AS5X47U_SPI_DRIVER_H

#include "stm32h7xx_hal.h" // Need this for SPI communications via HAL

// NOTE - This encoder works on mode=1 for SPI, so CPOL=0, CPHA=1

/* DEFINES */


/* REGISTERS 
NOTE -  Register addresses need to be 14bit for SPI transactions but they are 16bit here
        Just overwrite the first two most significant bits (at the left of the number; always 00 by default)
*/
#define NOP             0x0000  // No description
#define ERRFL           0x0001  // Error Register
#define PROG            0x0003  // Programming register
#define DIA             0x3FF5  // DIAGNOSTIC
#define AGC             0x3FF9  // Automatic Gain Control (AGC) value - 8 bit
#define SINE_DATA       0x3FFA  // Raw digital sine channel data
#define COSINE_DATA     0x3FFB  // Raw digital cosine channel data
#define VEL             0x3FFC  // Velocity - 14 bit signed integer
#define MAG             0x3FFD  // CORDIC magnitude
#define ANGLEUNC        0x3FFE  // Measured angle without dynamic angle error compensation
#define ANGLECOM        0x3FFF  // Measured angle with dynamic angle error compensation
#define ECC_Checksum    0x00D1  // ECC Checksum calculated based on actual register setting - bits 6:0


/* ENUMS */


/* Structs */
typedef struct AS5x47U {
    // Port and Pin information
    SPI_HandleTypeDef* hspi;
    uint8_t enc_addr;
    GPIO_TypeDef* CS_port;  // Format of GPIOx, where x = A,B,C,D,E,F
    uint16_t CS_pin;        // Format of GPIO_PIN_x, where x = 0 to 16

    // Configuration information
    uint16_t rxBuffer16;    
    uint8_t rxBuffer[3]; // NOTE - 3 bytes in length for 24 bit transactions specifically
    // Actual data stored away
    float velocity;
    float angle_comp;
    float angle_uncomp;
    int16_t CORDIC_mag;

    // Calibration information
    

    // CRC variables
    uint32_t last_crc;
    uint8_t crcPoly;
    uint8_t crcTable[256];
    CRC_HandleTypeDef* hcrc;

    // Error register
    uint8_t warningBit;
    uint8_t errorBit;
    uint8_t err_reg;



} AS5x47U;


/* Initialisation Functions */
HAL_StatusTypeDef AS5x47U_init(AS5x47U* enc_ptr, SPI_HandleTypeDef* hspi, GPIO_TypeDef* enc_CS_port, uint16_t enc_CS_pin, CRC_HandleTypeDef* hcrc);

/* Data Acquistion Functions */
HAL_StatusTypeDef AS5x47U_readPositionDAE(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_readPositionNoDAE(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_readVelocity(AS5x47U* enc_ptr); 
HAL_StatusTypeDef AS5x47U_readERRFL(AS5x47U* enc_ptr);

/* Low Level Functions */
// NOTE - SPI commands here work with 24bit frames for CRC 8bit checks + we don't need the speed of 16bit frames
HAL_StatusTypeDef AS5x47U_readRegister(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t* output);
HAL_StatusTypeDef _AS5x47U_readRegisterNOP(AS5x47U* enc_ptr);
HAL_StatusTypeDef _AS5x47U_readRegisterNOP16(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_readRegister16(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t* output);

HAL_StatusTypeDef AS5x47U_writeRegister(AS5x47U* enc_ptr, uint16_t reg_addr, uint16_t input);
HAL_StatusTypeDef AS5x47U_calcCRC(AS5x47U* enc_ptr, uint16_t crcData); // Calculation based on bits 23:8 -> Page 27 of 61: CRC Checksum
void AS5x47U_calcCRC_new(AS5x47U* enc_ptr, uint8_t* bytes, int numOfBytes);
void AS5x47U_calcCRCTable(AS5x47U* enc_ptr);

void CRC_calc_HAL(AS5x47U* enc_ptr, uint8_t* bytes, size_t numOfBytes);
#endif
