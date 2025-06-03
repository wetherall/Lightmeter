/**
 * I2C Communication Implementation
 * * Implements I2C master communication functions for the ATtiny3217.
 * Includes error checking for NACK conditions.
 */

 #include <avr/io.h>
 #include "i2c.h"
 
 // Defines for I2C timing based on F_CPU=4MHz for 100kHz SCL
 // TWI_BAUD = (F_CPU / (2 * F_SCL)) - 5
 // TWI_BAUD = (4000000 / (2 * 100000)) - 5 = (4000000 / 200000) - 5 = 20 - 5 = 15
 #define TWI_MBAUD_100KHZ 15 
 
 /**
  * @brief Initialize I2C communication.
  */
 void init_i2c(void) {
     /* Set up TWI peripheral for standard speed (100kHz) with F_CPU = 4MHz */
     TWI0.MBAUD = (uint8_t)TWI_MBAUD_100KHZ; 
     
     /* Enable TWI as master, enable smart mode (auto ACK/NACK) */
     TWI0.MCTRLA = TWI_ENABLE_bm | TWI_SMEN_bm;
     
     /* Force bus state to idle */
     TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
 }
 
 /**
  * @brief Write data to an I2C device.
  */
 uint8_t i2c_write(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length) {
     /* Send slave address with write bit (0) */
     TWI0.MADDR = (device_addr << 1) & ~0x01;
     
     /* Wait for address transmission to complete or error */
     while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm)));
 
     /* Check for NACK (address not acknowledged) or bus error/arbitration lost */
     if (TWI0.MSTATUS & (TWI_RXACK_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
         TWI0.MCTRLB = TWI_MCMD_STOP_gc; // Send stop condition
         return 1; // Error: Device not responding or bus error
     }
     
     /* Send register address */
     TWI0.MDATA = reg;
     
     /* Wait for data transmission to complete or error */
     while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm)));
 
     /* Check for NACK (register not acknowledged) or bus error */
     if (TWI0.MSTATUS & (TWI_RXACK_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
         TWI0.MCTRLB = TWI_MCMD_STOP_gc; // Send stop condition
         return 2; // Error: Register not acknowledged or bus error
     }
     
     /* Send all data bytes */
     for (uint8_t i = 0; i < length; i++) {
         TWI0.MDATA = data[i];
         
         /* Wait for data transmission to complete or error */
         while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm)));
         
         /* Check for NACK (data not acknowledged) or bus error */
         if (TWI0.MSTATUS & (TWI_RXACK_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
             TWI0.MCTRLB = TWI_MCMD_STOP_gc; // Send stop condition
             return 2; // Error: Data not acknowledged or bus error
         }
     }
     
     /* End transmission with a stop condition */
     TWI0.MCTRLB = TWI_MCMD_STOP_gc;
     
     return 0; // Success
 }
 
 /**
  * @brief Read data from an I2C device.
  */
 uint8_t i2c_read(uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t length) {
     /* Send slave address with write bit (0) to set register pointer */
     TWI0.MADDR = (device_addr << 1) & ~0x01;
     
     /* Wait for address transmission to complete or error */
     while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm)));
     
     /* Check for NACK (address not acknowledged) or bus error */
     if (TWI0.MSTATUS & (TWI_RXACK_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
         TWI0.MCTRLB = TWI_MCMD_STOP_gc;
         return 1; // Error: Device not responding (write phase)
     }
     
     /* Send register address */
     TWI0.MDATA = reg;
 
     /* Wait for data transmission to complete or error */
     while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm)));
 
     /* Check for NACK (register not acknowledged) or bus error */
     if (TWI0.MSTATUS & (TWI_RXACK_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
         TWI0.MCTRLB = TWI_MCMD_STOP_gc;
         return 2; // Error: Register not acknowledged
     }
     
     /* Send slave address with read bit (1) using repeated start */
     TWI0.MADDR = (device_addr << 1) | 0x01;
     
     /* Wait for address transmission to complete or error */
     while (!(TWI0.MSTATUS & (TWI_RIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm | TWI_RXACK_bm)));
 
     /* Check for NACK (address not acknowledged for read) or bus error */
     if (TWI0.MSTATUS & (TWI_RXACK_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
         TWI0.MCTRLB = TWI_MCMD_STOP_gc;
         return 3; // Error: Device not responding (read phase)
     }
     
     /* Read all data bytes */
     for (uint8_t i = 0; i < length; i++) {
         /* Wait for data byte to be received */
          while (!(TWI0.MSTATUS & (TWI_RIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)));
         
         data[i] = TWI0.MDATA;
         
         /* Send ACK for all bytes except the last one */
         if (i < length - 1) {
             TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc; // Send ACK
         } else {
             TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc; // Send NACK and then STOP
         }
     }
     
     return 0; // Success
 }
 