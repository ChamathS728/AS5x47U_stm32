/*
 * AS5x47U.h
 * AS5x47U Magnetic Encoder Driver over SPI
 *  Created on: Nov 10, 2023
 *      Author: Chamath Suraweera
 *
 */

#ifndef AS5X47U_SPI_DRIVER_H
#define AS5X47U_SPI_DRIVER_H

#include "stm32h7xx_hal.h" // Need this for I2C communications via HAL

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
    SPI_HandleTypeDef* hspi;
    uint8_t enc_addr;

    // Configuration information
    int16_t rxBuffer[3];    // NOTE - 3 bytes in length for 24 bit transactions specifically
    int8_t CRC;             // Only touch this variable when calculating CRC 

    // Actual data stored away
    int16_t velocity;
    int16_t angle_comp;
    int16_t CORDIC_mag;

    // Calibration information
    

} AS5x47U;


/* Initialisation Functions */


/* Data Acquistion Functions */
HAL_StatusTypeDef AS5x47U_readPosition(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_readVelocity(AS5x47U* enc_ptr); 

/* Low Level Functions */
// NOTE - SPI commands here work with 24bit frames for CRC 8bit checks + we don't need the speed of 16bit frames
HAL_StatusTypeDef AS5x47U_readRegister(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_readRegisters(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_writeRegister(AS5x47U* enc_ptr);
HAL_StatusTypeDef AS5x47U_calcCRC(AS5x47U* enc_ptr); // Calculation based on bits 23:8 -> Page 27 of 61: CRC Checksum


#endif