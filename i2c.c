/**
 * I2C Communication Implementation
 * 
 * This file contains the implementation of I2C communication functions
 * used throughout the light meter project.
 * 
 * Updated to include proper error checking for NACK conditions,
 * which occur when a device doesn't acknowledge a transmission.
 */

#include <avr/io.h>
#include "i2c.h"

/**
 * Initialize I2C communication
 */
void init_i2c(void) {
    /* Set up TWI peripheral for standard speed (100kHz) */
    TWI0.MBAUD = 16;
    
    /* Enable TWI as master */
    TWI0.MCTRLA = TWI_ENABLE_bm;
    
    /* Set up for Smart Mode (automatic ACK/NACK handling) */
    TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
}

/**
 * Write data to an I2C device
 * 
 * This function now includes proper error checking for NACK conditions.
 * A NACK (Not Acknowledge) indicates that:
 * - The device is not present on the bus
 * - The device is busy and cannot respond
 * - There's a communication error
 * 
 * Returns:
 *   0 if successful
 *   1 if NACK received during address phase
 *   2 if NACK received during data phase
 */
uint8_t i2c_write(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length) {
    /* Start transmission */
    TWI0.MADDR = (device_addr << 1) | 0; // Write operation
    
    /* Wait for transmission to start */
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    
    /* Check for NACK after sending device address */
    if (TWI0.MSTATUS & TWI_RXACK_bm) {
        /* Device did not acknowledge - stop and return error */
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return 1;  // Error: Device not responding
    }
    
    /* Send register address */
    TWI0.MDATA = reg;
    
    /* Wait for data to be sent */
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    
    /* Check for NACK after sending register address */
    if (TWI0.MSTATUS & TWI_RXACK_bm) {
        /* Register address not acknowledged - stop and return error */
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return 2;  // Error: Register not acknowledged
    }
    
    /* Send all data bytes */
    for (uint8_t i = 0; i < length; i++) {
        TWI0.MDATA = data[i];
        
        /* Wait for data to be sent */
        while (!(TWI0.MSTATUS & TWI_RIF_bm));
        
        /* Check for NACK after each data byte */
        if (TWI0.MSTATUS & TWI_RXACK_bm) {
            /* Data not acknowledged - stop and return error */
            TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            return 2;  // Error: Data not acknowledged
        }
    }
    
    /* End transmission */
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
    
    return 0;  // Success
}

/**
 * Read data from an I2C device
 * 
 * This function now includes proper error checking for NACK conditions
 * during both the write phase (register selection) and read phase.
 * 
 * Returns:
 *   0 if successful
 *   1 if NACK received during address phase
 *   2 if NACK received during register write phase
 *   3 if NACK received during read restart phase
 */
uint8_t i2c_read(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length) {
    /* Start transmission for register selection */
    TWI0.MADDR = (device_addr << 1) | 0; // Write operation
    
    /* Wait for transmission to start */
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    
    /* Check for NACK after sending device address (write) */
    if (TWI0.MSTATUS & TWI_RXACK_bm) {
        /* Device did not acknowledge - stop and return error */
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return 1;  // Error: Device not responding
    }
    
    /* Send register address */
    TWI0.MDATA = reg;
    
    /* Wait for data to be sent */
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    
    /* Check for NACK after sending register address */
    if (TWI0.MSTATUS & TWI_RXACK_bm) {
        /* Register address not acknowledged - stop and return error */
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return 2;  // Error: Register not acknowledged
    }
    
    /* Send restart for read operation */
    TWI0.MADDR = (device_addr << 1) | 1; // Read operation
    
    /* Wait for restart to complete */
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    
    /* Check for NACK after sending device address (read) */
    if (TWI0.MSTATUS & TWI_RXACK_bm) {
        /* Device did not acknowledge read request - stop and return error */
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;
        return 3;  // Error: Device not responding to read
    }
    
    /* Read all data bytes */
    for (uint8_t i = 0; i < length; i++) {
        /* For last byte, send NACK after */
        if (i == length - 1) {
            TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_RECVTRANS_gc;
        } else {
            TWI0.MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;
        }
        
        /* Read byte from TWI data register */
        data[i] = TWI0.MDATA;
        
        /* Wait for data to be received (except for last byte) */
        if (i < length - 1) {
            while (!(TWI0.MSTATUS & TWI_RIF_bm));
        }
    }
    
    /* End transmission */
    TWI0.MCTRLB = TWI_MCMD_STOP_gc;
    
    return 0;  // Success
}