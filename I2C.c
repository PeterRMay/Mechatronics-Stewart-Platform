/*
 * Example of using the I2C block
 *
 * Copyright (c) 2015,
 * National Instruments Corporation.
 * All rights reserved.
 */

#include <stdio.h>
#include <time.h>

/*
 * Include the myRIO header file.
 * The target type must be defined in your project, as a stand-alone #define,
 * or when calling the compiler from the command-line.
 */
#include "MyRio.h"
#include "I2C.h"

/*
 * Declare the myRIO NiFpga_Session so that it can be used by any function in
 * this file. The variable is actually defined in myRIO.c.
 *
 * This removes the need to pass the myrio_session around to every function and
 * only has to be declared when it is being used.
 */
extern NiFpga_Session myrio_session;


/**
 * I2C control options.
 */
typedef enum
{
    I2c_Rx_Tx = 1,   /* 0b00000001 */
    I2c_Start = 2,   /* 0b00000010 */
    I2c_Stop = 4,    /* 0b00000100 */
    I2c_Ack = 8      /* 0b00001000 */
} I2c_ControlBits;


/**
 * I2C status options.
 */
typedef enum
{
    I2c_Busy = 1,         /* 0b00000001 */
    I2c_Error = 2,          /* 0b00000010 */
    I2c_Address_Nak = 4,  /* 0b00000100 */
    I2c_Data_Nak = 8,     /* 0b00001000 */
    I2c_In_Use = 16,      /* 0b00010000 */
    I2c_Bus_Busy = 32     /* 0b00100000 */
} I2c_StatusBits;


/**
 * Configure the I2C block.
 * Set options for the I2C configuration register.
 *
 * @param[in]  channel  A struct containing the registers on the I2C channel to
 *                      modify.
 * @param[in]  settings Settings configured on the register
 */
void I2c_Configure(MyRio_I2c* channel, I2c_ConfigureSettings settings)
{
    NiFpga_Status status;

    status = NiFpga_WriteU8(myrio_session, channel->cnfg, settings);

    MyRio_ReturnIfNotSuccess(status,
            "Problem configuring the I2C channel!");
}


/**
 * Set the speed of the I2C block.
 *
 * Standard mode (100kbps) = 187.
 * Fast mode (400kbps) = 51.
 *
 * These values are calculated using the formula:
 *   f_SCL = f_clk / 2 [(2 * CNTR) - 4]
 *
 * where:
 *   f_SCL = the desired frequency of the I2C transmission
 *   f_clk = the frequency of the myRIO FPGA clock (40 MHz default)
 *
 * This formula and its rationale can be found in the documentation.
 *
 * @param[in]  channel  A struct containing the registers on the I2C channel to
 *                      modify.
 * @param[in]  speed    The I2C speed configured on the I2C channel
 */
void I2c_Counter(MyRio_I2c* channel, uint8_t speed)
{
    NiFpga_Status status;

    status = NiFpga_WriteU8(myrio_session, channel->cntr, speed);

    MyRio_ReturnIfNotSuccess(status,
            "Problem setting the speed of the I2C channel!");
}


/**
 * Write a series of bytes to the I2C channel.
 *
 * @warning The data array being passed in must be at least as big as the
 *          number of bytes being written.
 *
 * @param[in]  channel   A struct containing the registers for the I2C channel
 *                       to be written to
 * @param[in]  address   The address of the I2C slave device
 * @param[in]  data      A pointer to an array holding the data to write
 * @param[in]  numBytes  The number of bytes to be written to the slave
 */
void I2c_Write(MyRio_I2c* channel, uint8_t address, uint8_t* data,
               uint32_t numBytes)
{
    const uint8_t timeoutDelay = 1;  /* 1 Second */
    NiFpga_Status status;
    uint32_t index;
    uint32_t lastIndex;
    uint8_t control;
    uint8_t stat;
    NiFpga_Bool busy;
    NiFpga_Bool timeout = NiFpga_False;
    NiFpga_Bool error = NiFpga_False;
    NiFpga_Bool addrNak;
    NiFpga_Bool dataNak;
    time_t startTime;
    time_t currentTime;

    /*
     * Fix the I2C address based on the write operation.
     *
     * Shift the address one bit to the left and clear bit0.
     */
    address = (address << 1) & 0xFE;

    /*
     * Set the address of the slave device.
     */
    status = NiFpga_WriteU8(myrio_session, channel->addr, address);

    /*
     * Check if writing to the address register was successful.
     *
     * If there was an error then the rest of the function cannot complete
     * correctly so print an error message to stdout and return from the
     * function early.
     */
    MyRio_ReturnIfNotSuccess(status,
            "Could not write to the I2C address register!");

    /*
     * Store the index value for the last byte.
     */
    lastIndex = numBytes - 1;

    /*
     * Repeat for all bytes. If there was a timeout or an error then don't continue to
     * send the rest of the data bytes so break out of the loop.
     */
    for (index = 0; (index < numBytes) && (!timeout & !error); index++)
    {
        /*
         * If there is only one byte then the I2C transmission is different
         * than if there is more than 1 byte to write.
         */
        if (numBytes == 1)
        {
            /*
             * Only one byte.
             * Generate start bit, send byte, and generate stop bit.
             */
            control = I2c_Start | I2c_Rx_Tx | I2c_Stop; /* 0b00000111 */
        }
        else
        {
            if (index == 0)
            {
                /*
                 * First byte.
                 * Generate start bit and send the first byte.
                 */
                control = I2c_Start | I2c_Rx_Tx; /* 0b00000011 */

            }
            else if (index == lastIndex)
            {
                /*
                 * Last byte.
                 * Send the last byte and generate stop bit
                 */
                control = I2c_Rx_Tx | I2c_Stop; /* 0b00000101 */
            }
            else
            {
                /*
                 * Intermediate byte.
                 * Send the byte.
                 */
                control = I2c_Rx_Tx; /* 0b00000001 */
            }
        }

        /*
         * Write the data byte to be transmitted.
         */
        status = NiFpga_WriteU8(myrio_session, channel->dato, data[index]);

        /*
         * Check if writing to the data out register was successful.
         *
         * If there was an error then the rest of the bytes cannot be sent so
         * so print an error message to stdout and break out of the loop. The
         * function cannot return early otherwise the I2C bus will not be
         * released and will be left in a bad state.
         */
        if (MyRio_IsNotSuccess(status))
        {
            error = NiFpga_True;
            MyRio_PrintStatus(status);
            printf("Could not write to the I2C data out register!");
            break;
        }

        /*
         * Write the control byte.
         */
        status = NiFpga_WriteU8(myrio_session, channel->cntl, control);
        if (MyRio_IsNotSuccess(status))
        {
            error = NiFpga_True;
            MyRio_PrintStatus(status);
            printf("Could not write to the I2C control register!");
            break;
        }

        /*
         * Start the I2C operation.
         *
         * Write a true value. The value will automatically be reset to false
         * after the I2C block starts the operation.
         */
        status = NiFpga_WriteBool(myrio_session, channel->go, NiFpga_True);
        if (MyRio_IsNotSuccess(status))
        {
            error = NiFpga_True;
            MyRio_PrintStatus(status);
            printf("Could not start the I2C operation!");
            break;
        }

        /*
         * Wait for the I2C operation to complete
         */
        startTime = time(NULL); /* Get the start time */
        do
        {
            /*
             * Get the status of the I2C block.
             */
            status = NiFpga_ReadU8(myrio_session, channel->stat, &stat);
            if (MyRio_IsNotSuccess(status))
            {
                error = NiFpga_True;
                MyRio_PrintStatus(status);
                printf("Could not read from the I2C status register!");
                break;
            }

            /*
             * Check the busy bit of the status register
             */
            busy = stat & I2c_Busy;

            if (!busy)
            {
                break;
            }
            /*
             * Check if one second has passed since the operation was started.
             * If it has then the operation has timed out
             */
            currentTime = time(NULL);
            timeout = (currentTime - startTime) >= timeoutDelay;
        }
        while (!timeout || !error);

        /*
         * If there was no error with the operation of the I2C block then check
         * if there was an error with the actual I2C transmission.
         */
        if (!timeout || !error)
        {
            error = stat & I2c_Error;
            if (error)
            {
                /*
                 * Pull out the values of the Address NAK and Data NAK bits.
                 */
                addrNak = (stat & I2c_Address_Nak) > 0;
                dataNak = (stat & I2c_Data_Nak) > 0;

                printf("Error during I2C Transmission!\n");
                printf("ADRNAK: %d, DATNAK: %d\n", addrNak, dataNak);
            }
        }
    }

    /*
     * If there was a timeout or an error then try to generate the stop bit to
     * release the I2C bus and leave it in a good state. If there was no error
     * then the stop bit would have already been generated so this step can be
     * skipped.
     */
    if (timeout || error)
    {
        status = NiFpga_WriteU8(myrio_session, channel->cntl, I2c_Stop);
        MyRio_ReturnIfNotSuccess(status,
                    "Could not write to the I2C control register!");
    }
}


/**
 * Read a series of bytes from the I2C channel.
 *
 * @warning The data array passed in must be big enough to accommodate
 *          the number of bytes being read.
 *
 * @param[in]      channel   A struct containing the registers for the I2C
 *                           channel to be read from
 * @param[in]      address   The address of the I2C slave device
 * @param[in,out]  data      A pointer to an array to fill with the data read
 * @param[in]      numBytes  The number of bytes to be read from the slave
 */
void I2c_Read(MyRio_I2c* channel, uint8_t address, uint8_t* data,
              uint32_t numBytes)
{
    const uint8_t timeoutDelay = 1;  /* 1 Second */
    NiFpga_Status status;
    uint32_t index;
    uint32_t lastIndex;
    uint8_t control;
    uint8_t stat;
    NiFpga_Bool busy;
    NiFpga_Bool timeout = NiFpga_False;
    NiFpga_Bool error = NiFpga_False;
    NiFpga_Bool addrNak;
    time_t startTime;
    time_t currentTime;

    /*
     * Fix the I2C address based on the read operation.
     *
     * Shift the address one bit to the left and set bit0.
     */
    address = (address << 1) | 0x01;

    /*
     * Set the address of the slave device
     */
    status = NiFpga_WriteU8(myrio_session, channel->addr, address);

    /*
     * Check if writing to the address register was successful.
     *
     * If there was an error then the rest of the function cannot complete
     * correctly so print an error message to stdout and return from the
     * function early.
     */
    MyRio_ReturnIfNotSuccess(status,
                "Could not write to the I2C address register!");

    /*
     * Store the index value for the last byte.
     */
    lastIndex = numBytes - 1;

    /*
     * Repeat for all bytes
     */
    for (index = 0; index < numBytes; index++)
    {
        /*
         * If there is only one byte then the I2C transmission is different
         * than if there is more than 1 byte to write.
         */
        if (numBytes == 1)
        {
            /*
             * Only one byte.
             * Generate start bit, receive byte, and generate stop bit.
             * Do not send an ACK since this the last byte.
             */
            control = I2c_Start | I2c_Rx_Tx | I2c_Stop; /* 0b00000111 */
        }
        else
        {
            if (index == 0)
            {
                /*
                 * First byte.
                 * Generate start bit and receive the first byte.
                 * Send an ACK so the slave knows it will have to send more
                 * data.
                 */
                control = I2c_Ack | I2c_Start | I2c_Rx_Tx; /* 0b00001011 */

            }
            else if (index == lastIndex)
            {
                /*
                 * Last byte.
                 * Receive last byte and generate stop bit.
                 * Do not send an ACK since this the last byte.
                 */
                control = I2c_Rx_Tx | I2c_Stop; /* 0b00000101 */
            }
            else
            {
                /*
                 * Intermediate byte.
                 * Send the byte.
                 * Send an ACK so the slave knows it will have to send more
                 * data.
                 */
                control = I2c_Ack | I2c_Rx_Tx; /* 0b00001001 */
            }
        }

        /*
         * Write the control byte.
         */
        status = NiFpga_WriteU8(myrio_session, channel->cntl, control);
        if (MyRio_IsNotSuccess(status))
        {
            error = NiFpga_True;
            MyRio_PrintStatus(status);
            printf("Could not write to the I2C control register!");
            break;
        }

        /*
         * Start the I2C operation.
         *
         * Write a true value. The value will automatically be reset to false
         * after the I2C block starts the operation.
         */
        status = NiFpga_WriteBool(myrio_session, channel->go, NiFpga_True);
        if (MyRio_IsNotSuccess(status))
        {
            error = NiFpga_True;
            MyRio_PrintStatus(status);
            printf("Could not start the I2C operation!");
            break;
        }

        /*
         * Wait for the I2C Operation to complete
         */
        startTime = time(NULL); /* Get the start time */
        do
        {
            /*
             * Get the status of the I2C block.
             */
            status = NiFpga_ReadU8(myrio_session, channel->stat, &stat);
            if (MyRio_IsNotSuccess(status))
            {
                error = NiFpga_True;
                MyRio_PrintStatus(status);
                printf("Could not read from the I2C status register!");
                break;
            }

            /*
             * Check the busy bit of the status register
             */
            busy = stat & I2c_Busy;

            if (!busy)
            {
                break;
            }

            /*
             * Check if one second has passed since the operation was started.
             * If it has then the operation has timed out
             */
            currentTime = time(NULL);
            timeout = (currentTime - startTime) >= timeoutDelay;
        }
        while (!timeout || !error);

        /*
         * If there was no error with the operation of the I2C block then check
         * if there was an error with the actual I2C transmission.
         */
        if (!timeout || !error)
        {
            error = stat & I2c_Error;
            if (error)
            {
                /*
                 * Pull out the value of the Address NAK. Data NAK is not
                 * important since this is a read operation.
                 */
                addrNak = (stat & I2c_Address_Nak) > 0;

                printf("Error during I2C Transmission!\n");
                printf("ADRNAK: %d\n", addrNak);
            }
            else
            {
                /*
                 * Read the data byte received.
                 */
                status = NiFpga_ReadU8(myrio_session, channel->dati,
                                &data[index]);

                /*
                 * Check if reading from the data in register was successful.
                 *
                 * If there was an error then the rest of the bytes cannot be
                 * received so print an error message to stdout and set error
                 * to break out of the loop. The function cannot return early
                 * otherwise the I2C bus will not be released and will be left
                 * in a bad state.
                 */
                if (MyRio_IsNotSuccess(status))
                {
                    error = NiFpga_True;
                    MyRio_PrintStatus(status);
                    printf("Could not read from the I2C Data In register!");
                }
            }
        }

        /*
         * If there was a timeout or an error then don't continue to send the
         * rest of the data bytes so break out of the loop.
         */
        if (timeout || error)
        {
            break;
        }
    }

    /*
     * If there was a timeout or an error then try to generate the stop bit to
     * release the I2C bus and leave it in a good state. If there was no error
     * then the stop bit would have already been generated so this step can be
     * skipped.
     */
    if (timeout || error)
    {
        status = NiFpga_WriteU8(myrio_session, channel->cntl, I2c_Stop);
        MyRio_ReturnIfNotSuccess(status,
                    "Could not write to the I2C control register!");
    }
}
