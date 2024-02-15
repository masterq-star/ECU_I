/*
 * Implements NVM read, write utilities utilising an external Microchip 24LC256
 * EEPROM via an I2C interface. Is required for operational data such as long term fuel mixture
 * recording as well as the ECU configuration data.
 *
 * A call to function nvTestEEPROMReady() will determine a) if I2C_INTERFACE is defined, and b) if the
 * EEPROM is connected and ready to be used. If yes, the EEPROM available bit is set in the ECU status word,
 * otherwise it remains cleared.
 *
 * The 24LC256 device has space for 32K bytes of data.
 *
 * For every data block written to the EEPROM, a 4 byte (32-bit) unsigned integer is appended to the data block
 * and space must be allocated on the device accordingly. This value holds the checksum of the data block. The checksum
 * is calculated by a simple summation of each byte in the data block, where each byte is treated as an unsigned 8 bit integer.
 *
 * The checksum is used to verify that the stored data on the device has been read back correctly. This provides a limited
 * integrity check of the stored data and the read operation.
 *
 *
 *

This software/firmware source code or executable program is copyright of
Just Technology (North West) Ltd (http://www.just-technology.co.uk) 2020

This software/firmware source code or executable program is provided as free software:
you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version. The license is available at https://www.gnu.org/licenses/gpl-3.0.html

The software/firmware source code or executable program is distributed in the hope that
it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.

*/


#include "nvm.h"
#include "main.h"
#include "global.h"
#include "stdio.h"
#include "string.h"

// define a timeout for each of the I2C operations used in the functions contained herein
#define I2C_TIMEOUT 1000

// the I2C address of the EEPROM device
#define EEPROM_I2C_ADDRESS 0xA0



// prototypes
HAL_StatusTypeDef nvEEPROMWrite(uint8_t *dataPtr, uint16_t destAddr, int nBytes, uint32_t *checksum);
HAL_StatusTypeDef nvEEPROMRead(uint8_t *destPtr, uint16_t srcAddr, int nBytes);
uint32_t nvCalcChecksum(uint8_t *dataPtr, int nBytes);


// returns 1 and sets EEPROM_AVAILABLE in the ecu status word if the EEPROM is available and ready
int nvTestEEPROMReady(){
	int result = 0;
	#ifdef I2C_INTERFACE
		if (HAL_I2C_IsDeviceReady(I2C_INTERFACE, EEPROM_I2C_ADDRESS | 1, 2, I2C_TIMEOUT) == HAL_OK) {
			result = 1;
			SET_EEPROM_AVAILABLE;
		}
	#endif
	return result;
}

/* Writes the specified number of data bytes to EEPROM to the EEPROM base address specified.
 * Appends a 4 byte integer to the number of data bytes that represents the data checksum.
 * The EEPROM base address is a 15 bit number specifying the on-device address, ranging from 0 to 32,767
 * Uses the "page write" method which writes up to 64 bytes in one operation.
 * Returns HAL_OK if the block write operation was successful, otherwise HAL_ERROR if any errors encountered.
 */
 HAL_StatusTypeDef nvEEPROMBlockWrite(uint8_t *dataPtr, uint16_t eepromAddress, int nBytes){

	// checksum
	uint32_t checksum = 0;

	// calculate number of 64 byte pages
	int pages = nBytes / 64;

	// calculate number additional bytes
	int additionalBytes = nBytes - pages * 64;

	// on-device destination address
	uint16_t addr = eepromAddress;

	// send data to EEPROM, 64 bytes at a time
	for (int i = 0; i < pages; i++){
		// attempt to send data
		if (nvEEPROMWrite(dataPtr, addr, 64, &checksum) != HAL_OK) {
			// error encountered
			return HAL_ERROR;
		}
		// increment pointers
		dataPtr += 64;
		addr += 64;
	}

	// send any remaining bytes in the data structure
	if (additionalBytes > 0) {
		if (nvEEPROMWrite(dataPtr, addr, additionalBytes, &checksum) != HAL_OK) {
			return HAL_ERROR;
		}
	}

	// grab a copy of the current checksum & get a byte pointer to it
	uint32_t checksumCpy = checksum;
	uint8_t *chkPtr = (uint8_t *) &checksumCpy;

	// send the checksum to the device
	return nvEEPROMWrite(chkPtr, eepromAddress + nBytes, 4, &checksum);
}


/*
 * Writes a maximum of 64 bytes to the EEPROM.
 * Returns HAL_OK if the write operation was successful.
 * If the write operation failed, returns HAL_ERROR and sets the EEPROM_WRITE_ERROR bit in the ECU status word.
 *
 */
HAL_StatusTypeDef nvEEPROMWrite(uint8_t *dataPtr, uint16_t destAddr, int numberBytes, uint32_t *checksum){

	HAL_StatusTypeDef result = HAL_OK;
	#ifdef I2C_INTERFACE
		// limit no of bytes to 64
		int nBytes = numberBytes <= 64 ? numberBytes : 64;

		// calculate the checksum of the supplied data
		for (int i = 0; i < nBytes; i++) {
			*checksum += *(dataPtr + i);
		}

		// temporary storage to hold up to 64 bytes
		// it's 66 long as the first two bytes hold the address of the device's memory
		uint8_t temp[66];

		// make sure the device is ready before initiating a write operation
		int eeWaitCount = 0;
		while (HAL_I2C_IsDeviceReady(I2C_INTERFACE, EEPROM_I2C_ADDRESS | 1, 1, I2C_TIMEOUT) != HAL_OK) {
			if (++eeWaitCount > 1000) {
				// too many attempts, set the ecu status and exit with error
				SET_EEPROM_WRITE_ERROR;
				return HAL_ERROR;
			}
		}

		// copy data to temp store
		memcpy(&temp[2], dataPtr, nBytes);

		// set the on-device address
		temp[0] = destAddr >> 8;
		temp[1] = destAddr & 0xFF;

		// try to send page (number of data bytes + 2 for the address)
		result = HAL_I2C_Master_Transmit(I2C_INTERFACE, EEPROM_I2C_ADDRESS, temp, nBytes + 2, I2C_TIMEOUT);
		if (result != HAL_OK) {
			SET_EEPROM_WRITE_ERROR;
		}
	#endif
	return result;
}


/* Reads the number of specified bytes from the EEPROM device into the memory address specifed from the device's source address.
 * Performs a check on the data block integrity by calculating the checksum of the data read and comparing this to the checsum on the device.
 * Returns HAL_OK if the data read operation was successful.
 * In the ECU STATUS WORD, sets EEPROM_READ_ERROR if the read operation failed.
 * Sets EEPROM_CHECKSUM_ERROR if the read operation was successful but the checksum did not match.
 */
HAL_StatusTypeDef nvEEPROMBlockRead(uint8_t *destPtr, uint16_t srcAddr, int nBytes) {

	#ifdef I2C_INTERFACE

		// wait until the device is ready
		int eeWaitCount = 0;
		while (HAL_I2C_IsDeviceReady(I2C_INTERFACE, EEPROM_I2C_ADDRESS | 1, 1, I2C_TIMEOUT) != HAL_OK) {
			if (++eeWaitCount > 1000) {
				// too many attempts, set the ecu status and exit with error
				SET_EEPROM_READ_ERROR;
				return HAL_ERROR;
			}
		}

		// read the data from EEPROM and send to the data structure
		if (nvEEPROMRead(destPtr, srcAddr, nBytes) != HAL_OK) {
			return HAL_ERROR;
		}

		// now get the checksum (using device sequential read)
		uint32_t checksum;
		uint8_t *checkPtr = (uint8_t *) &checksum;
		if (HAL_I2C_Master_Receive(I2C_INTERFACE, EEPROM_I2C_ADDRESS | 1, checkPtr, 4, I2C_TIMEOUT) != HAL_OK) {
			return HAL_ERROR;
		}

		// calculate the checksum of the restored data & compare with the checksum stored on the EEPROM device
		if (nvCalcChecksum(destPtr, nBytes) != checksum) {
			// checksum error
			SET_EEPROM_CHECKSUM_ERROR;
		}

	#endif

	return HAL_OK;
}

/* Reads the number of specified bytes from the EEPROM device into the memory address specifed from the device's source address.
 * Returns HAL_OK if the data read operation was successful.
 * In the ECU STATUS WORD, sets EEPROM_READ_ERROR if the read operation failed.
 */
HAL_StatusTypeDef nvEEPROMRead(uint8_t *destPtr, uint16_t srcAddr, int nBytes){
	// set on-device src address
	uint8_t addr[2] = { srcAddr >> 8, srcAddr & 0xFF };
	HAL_StatusTypeDef s = HAL_I2C_Master_Transmit(I2C_INTERFACE, EEPROM_I2C_ADDRESS, addr, 2, I2C_TIMEOUT);
	if (s != HAL_OK) {
		SET_EEPROM_READ_ERROR;
		return HAL_ERROR;
	}
	// read the data
	if (HAL_I2C_Master_Receive(I2C_INTERFACE, EEPROM_I2C_ADDRESS | 1, destPtr, nBytes, I2C_TIMEOUT) != HAL_OK) {
		SET_EEPROM_READ_ERROR;
		return HAL_ERROR;
	}
	return HAL_OK;
}



// calculates the checksum of a number of data bytes
uint32_t nvCalcChecksum(uint8_t *dataPtr, int nBytes){
	uint32_t checksum = 0;
	for (int i = 0; i < nBytes; i++){
		checksum += *dataPtr++;
	}
	return checksum;
}




/*+++REVISION_HISTORY+++
1) 05 Jan 2021 Preparing to remove Flash as a NVM data store. Future updates will utilise only external EEPROM devices.
2) 05 Jan 2021 Checksum capability added to EEPROM Block Read & Block Write operations.
3) 09 Jan 2021 Flash read/write operations removed.
+++REVISION_HISTORY_ENDS+++*/
