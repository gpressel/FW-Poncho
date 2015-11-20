/* Copyright 2015, Dominguez Shocron, Marcos
 * Copyright 2015, Greggio, Alejandro
 * Copyright 2015, Halter, Christian
 * Copyright 2015, Pressel Coretto, Germán
 * Copyright 2015, Sosa, Carina
 * All right reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _CIAADRIVERSPI_H_
#define _CIAADRIVERSPI_H_
/** \brief CIAA SPI driver header file
 **
 ** This files contains the header file of the CIAA SPI driver
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Drivers CIAA Drivers
 ** @{ */
/** \addtogroup SPI SPI Drivers
 ** @{ */

 /*
 * Initials     Name
 * ---------------------------
 * DoMa         Dominguez Shocron, Marcos
 * GrAl         Greggio, Alejandro
 * HaCh         Halter,Christian
 * PrGe		    Pressel Coretto, Germán
 * SoCa			Sosa, Carina
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20151120 v0.0.1  initial version
 */

/*==================[inclusions]=============================================*/
#include "ciaaPOSIX_stdint.h"
#include "ciaaSerialDevices.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** \brief open the SPI device
 **
 ** \param[in] path path of the device to be opened
 ** \param[in] device device to be opened
 ** \param[in] oflag may take one of the following values:
 **               O_RDONLY: opens files to read only
 **               O_WRONLY: opens files to write only
 **               O_RDWR: opens file to read and write
 ** \return NULL if an error occurs, in other case the address of the opened
 **         device.
 **/
extern ciaaDevices_deviceType * ciaaDriverSPI_open(char const * path,
      ciaaDevices_deviceType * device, uint8_t const oflag);

/** \brief close the SPI device
 **
 **
 ** \param[in] device pointer to device
 ** \return    -1 if failed, 0 if success.
 **/
extern int32_t ciaaDriverSPI_close(ciaaDevices_deviceType const * const device);

/** \brief controls the SPI device
 **
 ** Performs special control of a SPI device
 **
 ** \param[in] device pointer to the device
 ** \param[in] request type of the request, depends on the device
 ** \param[in] param
 ** \return    a negative value if failed, a positive value
 **            if success.
 **/
extern int32_t ciaaDriverSPI_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param);

/** \brief read from a SPI device
 **
 ** Reads nbyte from the SPI device device in buf.
 **
 ** \param[in]  device  pointer to the device to be read
 ** \param[out] buf     buffer to store the read data
 ** \param[in]  nbyte   count of bytes to be read
 ** \return     the count of read bytes is returned
 **/
extern ssize_t ciaaDriverSPI_read(ciaaDevices_deviceType const * const device, uint8_t * const buffer, size_t const size);

/** \brief writes to a SPI device
 **
 ** Writes nbyte to the device device from the buffer buf
 **
 ** \param[in]  device  device to be written
 ** \param[in]  buf     buffer with the data to be written
 ** \param[in]  nbyte   count of bytes to be written
 ** \return     the count of bytes written
 **/
extern ssize_t ciaaDriverSPI_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size);

/** \brief initialize the SPI deriver
 **
 ** Is called at system startup, the driver shall register all available SPI
 ** devices.
 **/
extern void ciaaDriverSPI_init(void);
/**
 * @brief	Set up the SSP frame format
 * @param	pSSP		: The base of SSP peripheral on the chip
 * @param	bits		: The number of bits transferred in each frame, should be SSP_BITS_4 to SSP_BITS_16
 * @param	frameFormat	: Frame format, should be :
 *							- SSP_FRAMEFORMAT_SPI
 *							- SSP_FRAME_FORMAT_TI
 *							- SSP_FRAMEFORMAT_MICROWIRE
 * @param	clockMode	: Select Clock polarity and Clock phase, should be :
 *							- SSP_CLOCK_CPHA0_CPOL0
 *							- SSP_CLOCK_CPHA0_CPOL1
 *							- SSP_CLOCK_CPHA1_CPOL0
 *							- SSP_CLOCK_CPHA1_CPOL1
 * @return	 Nothing
 * @note	Note: The clockFormat is only used in SPI mode
 */
void Chip_SSP_SetFormat(LPC_SSP_T *pSSP, uint32_t bits, uint32_t frameFormat, uint32_t clockMode)
{
	pSSP->CR0 = (pSSP->CR0 & ~0xFF) | bits | frameFormat | clockMode;
}


/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _CIAADRIVERSPI_H_ */


