/*
 * I2C.h
 *
 * Copyright (c) 2015,
 * National Instruments Corporation.
 * All rights reserved.
 */

#ifndef I2C_h_
#define I2C_h_

#include "MyRio.h"

#if NiFpga_Cpp
extern "C" {
#endif


/**
 * I2C configuration register options.
 */
typedef enum
{
    I2c_Disabled = 0x00,
    I2c_Enabled = 0x01
} I2c_ConfigureSettings;


/**
 * Registers for a particular I2C.
 */
typedef struct
{
    uint32_t addr; /**< I2C slave address register*/
    uint32_t cnfg; /**< I2C configuration register*/
    uint32_t cntl; /**< I2C control register*/
    uint32_t cntr; /**< I2C counter register*/
    uint32_t stat; /**< I2C status register*/
    uint32_t dato; /**< I2C data out register*/
    uint32_t dati; /**< I2C data in register*/
    uint32_t go;   /**< I2C execution register*/
} MyRio_I2c;


/**
 * Configure the I2C block.
 */
void I2c_Configure(MyRio_I2c* channel, I2c_ConfigureSettings settings);


/**
 * Set the speed of the I2C block.
 */
void I2c_Counter(MyRio_I2c* channel, uint8_t speed);


/**
 * Write a series of bytes to the I2C channel.
 */
void I2c_Write(MyRio_I2c* channel, uint8_t address, uint8_t* data,
               uint32_t numBytes);


/**
 * Read a series of bytes from the I2C channel.
 */
void I2c_Read(MyRio_I2c* channel, uint8_t address, uint8_t* data,
              uint32_t numBytes);

#if NiFpga_Cpp
}
#endif

#endif /* I2C_h_ */
