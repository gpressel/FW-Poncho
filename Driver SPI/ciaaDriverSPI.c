/* Copyright 2015, Grupo Practicas_Embebidos - Facultad de Ingenieria(UNER)
 *
 * All rights reserved. 
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

/** \brief CIAA SPI Driver for LPC4337
 **
 ** Implements the SPI Driver for LPC4337
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
 * PrGe		    Pressel Coretto, German
 * SoMa			Sosa, Carina
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20151120 v0.0.1    initial version
 */

/*==================[inclusions]=============================================*/
#include "ciaaDriverSPI.h"
#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_stdio.h"
#include "chip.h"
#include "os.h"

/*==================[macros and definitions]=================================*/

typedef struct  {
   ciaaDevices_deviceType * const * const devices;
   uint8_t countOfDevices;
} ciaaDriverConstType;

#define SPI_FIFO_SIZE       (8)
#define MASK_BITS 0xF
#define MASK_CPHA_CPOL 0xC0
#define ciaaPOSIX_IOCTL_SET_CONFIG  10
#define ciaaPOSIX_IOCTL_SET_ENABLE_INTERRUPT 11

typedef struct {
   uint8_t hwbuf[SPI_FIFO_SIZE];
   uint8_t rxcnt;
} ciaaDriverSPIControl;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/** \brief Buffers */
ciaaDriverSPIControl SPIControl;

/** \brief Device for SPI 0 */
static ciaaDevices_deviceType ciaaDriverSPI_device = {
   "spi/0",               /** <= driver name */
   ciaaDriverSPI_open,    /** <= open function */
   ciaaDriverSPI_close,   /** <= close function */
   ciaaDriverSPI_read,    /** <= read function */
   ciaaDriverSPI_write,   /** <= write function */
   ciaaDriverSPI_ioctl,   /** <= ioctl function */
   NULL,                  /** <= seek function is not provided */
   NULL,                  /** <= uper layer */
   &(SPIControl),         /** <= layer */
   LPC_SPI                /** <= lower layer */
};


static ciaaDevices_deviceType * const ciaaSPIDevices = &ciaaDriverSPI_device;

static ciaaDriverConstType const ciaaDriverSPIConst = {
   ciaaSPIDevices,
   1
};



/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void ciaaDriverSPI_rxIndication(ciaaDevices_deviceType const * const device, uint32_t const nbyte)
{
    /*receive the data and forward to upper layer*/
   ciaaSerialDevices_rxIndication(device->upLayer, nbyte);
}

static void ciaaDriverSPI_txConfirmation(ciaaDevices_deviceType const * const device)
{
    /*receive the data and forward to upper layer*/
   ciaaSerialDevices_txConfirmation(device->upLayer, 1 );
}

static void ciaaDriverSPI_hwInit(void)
{
   /* SPI initialization */
   Chip_SSP_Init(LPC_SSP1);			/*El SPI se implementa por los pines de SSP1*/

   Chip_SCU_PinMux(0xF, 4, MD_PLN, FUNC0);   /* PF_4: SSP1_SCK */
   Chip_SCU_PinMux(1, 3, MD_PLN, FUNC5); 	 /* P1_3: SSP1_MISO */
   Chip_SCU_PinMux(1, 4, MD_PLN, FUNC5); 	 /* P1_4: SSP1_MOSI */

   Chip_SSP_Int_Disable(LPC_SSP1);
   Chip_SSP_Enable(LPC_SSP1);
   Chip_SSP_SetFormat(LPC_SSP1,SSP_BITS_8,SSP_FRAMEFORMAT_SPI,SSP_CLOCK_CPHA0_CPOL0);
   Chip_SSP_SetBitRate(LPC_SSP1, 1000000);
}

/*==================[external functions definition]==========================*/
extern ciaaDevices_deviceType * ciaaDriverSPI_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   /* Restart FIFOS */
   Chip_SSP_Int_FlushData((LPC_SSP_T *)device->loLayer);
   
   /* enable interrupt */
   Chip_SSP_Int_Enable((LPC_SSP_T *)device->loLayer);

   return device;
}

extern int32_t ciaaDriverSPI_close(ciaaDevices_deviceType const * const device)
{
   /* disable interrupt */
   Chip_SSP_Int_Disable((LPC_SSP_T *)device->loLayer);
   return 0;
}

extern int32_t ciaaDriverSPI_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   int32_t ret = -1;

   if(device == ciaaDriverSPIConst.devices)
   {
      switch(request)
      {
         case ciaaPOSIX_IOCTL_STARTTX:
            /* disable IRQ (TX) */
            Chip_SSP_Int_Disable((LPC_SSP_T *)device->loLayer);
            /* this one calls write */
            ciaaDriverSPI_txConfirmation(device);
            /* enable IRQ (TX) */
            Chip_SSP_Int_Enable((LPC_SSP_T *)device->loLayer);
            ret = 0;
            break;

         case ciaaPOSIX_IOCTL_SET_BAUDRATE:
            Chip_SSP_SetBitRate((LPC_SSP_T *)device->loLayer,  (int32_t)param);
            ret = 0;
            break;

         case ciaaPOSIX_IOCTL_SET_CONFIG:
        	 Chip_SSP_SetFormat((LPC_SSP_T *)device->loLayer, (uint32_t)param & MASK_BITS, SSP_FRAMEFORMAT_SPI, (uint32_t)param & MASK_CPHA_CPOL);
        	 break;

         case ciaaPOSIX_IOCTL_SET_ENABLE_INTERRUPT:
            if((bool)param == false)
            {
               /* disable IRQ */
            	Chip_SSP_Int_Disable((LPC_SSP_T *)device->loLayer);
            }
            else
            {
               /* enable IRQ */
            	Chip_SSP_Int_Enable((LPC_SSP_T *)device->loLayer);
            }
            break;
      }
   }
   return ret;
}

extern ssize_t ciaaDriverSPI_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   ssize_t ret = -1;
   ciaaDriverSPIControl * pSPIControl;
   uint8_t i;

   if(size != 0)
   {
      if(device == ciaaDriverSPIConst.devices)
      {
         pSPIControl = (ciaaDriverSPIControl *)device->layer;

         if(size > pSPIControl->rxcnt)
         {
            /* buffer has enough space */
            ret = pSPIControl->rxcnt;
            pSPIControl->rxcnt = 0;
         }
         else
         {
            /* buffer hasn't enough space */
            ret = size;
            pSPIControl->rxcnt -= size;
         }
         for(i = 0; i < ret; i++)
         {
            buffer[i] = pSPIControl->hwbuf[i];
         }
         if(pSPIControl->rxcnt != 0)
         {
            /* We removed data from the buffer, it is time to reorder it */
            for(i = 0; i < pSPIControl->rxcnt ; i++)
            {
               pSPIControl->hwbuf[i] = pSPIControl->hwbuf[i + ret];
            }
         }
      }
   }
   return ret;
}

extern ssize_t ciaaDriverSPI_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   ssize_t ret = 0;

   if(device == ciaaDriverSPIConst.devices)
   {
      while((Chip_SSP_GetStatus((LPC_SSP_T *)device->loLayer), SSP_STAT_TNF) &&
    		  (ret < size))
      {
         /* send first byte */
    	  Chip_SSP_WriteFrames_Blocking((LPC_SSP_T *)device->loLayer, buffer[ret],1);
         /* bytes written */
         ret++;
      }
   }
   return ret;
}

void ciaaDriverUart_init(void)
{
   uint8_t loopi;

   /* init hardware */
   ciaaDriverSPI_hwInit();

   /* add SPI driver to the list of devices */
   for(loopi = 0; loopi < ciaaDriverSPIConst.countOfDevices; loopi++) {
      /* add each device */
      ciaaSerialDevices_addDriver(ciaaDriverSPIConst.devices[loopi]);
   }
}

/*==================[interrupt handlers]=====================================*/
ISR(SPI0_IRQHandler)
{
    if(Chip_SSP_GetStatus(LPC_SSP1,SSP_STAT_RNE))
	{
      do
      {
         SPIControl.hwbuf[SPIControl.rxcnt] = Chip_SSP_ReadFrames_Blocking(LPC_SSP1);
         SPIControl.rxcnt++;
      }while(Chip_SSP_GetStatus(LPC_SSP1,SSP_STAT_RNE) &&
             (SPIControl.rxcnt < SPI_FIFO_SIZE));

      ciaaDriverSPI_rxIndication(&ciaaDriverSPI_device, SPIControl.rxcnt);
   }
   if(!Chip_SSP_GetStatus(LPC_SSP1,SSP_STAT_TFE) && (Chip_SSP_GetIntStatus(LPC_SSP1) & SSP_TXMIS))
   {
      /* tx confirmation, 1 byte sent */
      ciaaDriverSPI_txConfirmation(&ciaaDriverSPI_device);

      if(Chip_SSP_GetStatus(LPC_SSP1,SSP_STAT_TFE))
           {  /* There is not more bytes to send, disable irq */
    	  Chip_SSP_Int_Disable(LPC_SSP1);
           }
   }
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

