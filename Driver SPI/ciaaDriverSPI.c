/* Copyright 2014, Pablo Ridolfi (UTN-FRBA)
 * Copyright 2014, Juan Cecconi
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

/** \brief CIAA UART Driver for LPC4337
 **
 ** Implements the UART Driver for LPC4337
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Drivers CIAA Drivers
 ** @{ */
/** \addtogroup UART UART Drivers
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * PaRi         Pablo Ridolfi
 * JuCe         Juan Cecconi
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20140731 v0.0.1   PR first functional version
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

#define SPI_RX_FIFO_SIZE       (16)

typedef struct {
   uint8_t hwbuf[SPI_RX_FIFO_SIZE];
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
   NULL,                   /** <= seek function is not provided */
   NULL,                   /** <= uper layer */
   &(SPIControl),      /** <= layer */
   LPC_SPI              /** <= lower layer */
};


static ciaaDevices_deviceType * const ciaaSPIDevices = &ciaaDriverSPI_device;

static ciaaDriverConstType const ciaaDriverSPIConst = {
   ciaaSPIDevices,
   1
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/*static void ciaaDriverSPI_rxIndication(ciaaDevices_deviceType const * const device, uint32_t const nbyte)
{
    receive the data and forward to upper layer
   ciaaSerialDevices_rxIndication(device->upLayer, nbyte);
}

static void ciaaDriverSPI_txConfirmation(ciaaDevices_deviceType const * const device)
{
    receive the data and forward to upper layer
   ciaaSerialDevices_txConfirmation(device->upLayer, 1 );
}*/

static void ciaaDriverSPI_hwInit(void)
{
   /* SPI initialization */
   Chip_SSP_Init(LPC_SSP1);			/*El SPI se implementa por los pines de SSP1*/
   Chip_SSP_SetBitRate(LPC_SSP1, 1000000);



   Chip_SCU_PinMux(0xF, 4, MD_PLN, FUNC0);   /* PF_4: SSP1_SCK */
   Chip_SCU_PinMux(1, 3, MD_PLN, FUNC5); 	 /* P1_3: SSP1_MISO */
   Chip_SCU_PinMux(1, 4, MD_PLN, FUNC5); 	 /* P1_4: SSP1_MOSI */

   Chip_SSP_Int_Disable(LPC_SSP1);
   Chip_SSP_Enable(LPC_SSP1);
   Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);
}

/*==================[external functions definition]==========================*/
extern ciaaDevices_deviceType * ciaaDriverSPI_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag)
{
   /* Restart FIFOS: set Enable, Reset content, set trigger level */
   //Chip_UART_SetupFIFOS((LPC_USART_T *)device->loLayer, UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | UART_FCR_TRG_LEV0);
   Chip_SSP_Int_FlushData((LPC_SSP_T *)device->loLayer);
   /* dummy read */
   //Chip_UART_ReadByte((LPC_USART_T *)device->loLayer);

   /* enable rx interrupt */
   //Chip_UART_IntEnable((LPC_USART_T *)device->loLayer, UART_IER_RBRINT);
   Chip_SSP_Int_Enable((LPC_SSP_T *)device->loLayer);

   return device;
}

extern int32_t ciaaDriverUart_close(ciaaDevices_deviceType const * const device)
{
   /* disable tx and rx interrupt */
   Chip_SSP_Int_Disable((LPC_SSP_T *)device->loLayer);
   return 0;
}

extern int32_t ciaaDriverUart_ioctl(ciaaDevices_deviceType const * const device, int32_t const request, void * param)
{
   int32_t ret = -1;

   if(device == ciaaDriverSPIConst.devices)
   {
      switch(request)
      {
         case ciaaPOSIX_IOCTL_STARTTX:
            /* disable THRE irq (TX) */
            Chip_UART_IntDisable((LPC_USART_T *)device->loLayer, UART_IER_THREINT);
            /* this one calls write */
            ciaaDriverUart_txConfirmation(device);
            /* enable THRE irq (TX) */
            Chip_UART_IntEnable((LPC_USART_T *)device->loLayer, UART_IER_THREINT);
            ret = 0;
            break;

         case ciaaPOSIX_IOCTL_SET_BAUDRATE:
            ret = Chip_UART_SetBaud((LPC_USART_T *)device->loLayer,  (int32_t)param);
            break;

         case ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL:
            Chip_UART_SetupFIFOS((LPC_USART_T *)device->loLayer,  UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | (int32_t)param);
            break;

         case ciaaPOSIX_IOCTL_SET_ENABLE_TX_INTERRUPT:
            if((bool)(intptr_t)param == false)
            {
               /* disable THRE irq (TX) */
               Chip_UART_IntDisable((LPC_USART_T *)device->loLayer, UART_IER_THREINT);
            }
            else
            {
               /* enable THRE irq (TX) */
               Chip_UART_IntEnable((LPC_USART_T *)device->loLayer, UART_IER_THREINT);
            }
            break;

         case ciaaPOSIX_IOCTL_SET_ENABLE_RX_INTERRUPT:
            if((bool)(intptr_t)param == false)
            {
               /* disable RBR irq (RX) */
               Chip_UART_IntDisable((LPC_USART_T *)device->loLayer, UART_IER_RBRINT);
            }
            else
            {
               /* enable RBR irq (RX) */
               Chip_UART_IntEnable((LPC_USART_T *)device->loLayer, UART_IER_RBRINT);
            }
            break;
      }
   }
   return ret;
}

extern ssize_t ciaaDriverUart_read(ciaaDevices_deviceType const * const device, uint8_t* buffer, size_t const size)
{
   ssize_t ret = -1;
   ciaaDriverUartControl * pUartControl;
   uint8_t i;

   if(size != 0)
   {
      if((device == ciaaDriverUartConst.devices[0]) ||
         (device == ciaaDriverUartConst.devices[1]) ||
         (device == ciaaDriverUartConst.devices[2]) )
      {
         pUartControl = (ciaaDriverUartControl *)device->layer;

         if(size > pUartControl->rxcnt)
         {
            /* buffer has enough space */
            ret = pUartControl->rxcnt;
            pUartControl->rxcnt = 0;
         }
         else
         {
            /* buffer hasn't enough space */
            ret = size;
            pUartControl->rxcnt -= size;
         }
         for(i = 0; i < ret; i++)
         {
            buffer[i] = pUartControl->hwbuf[i];
         }
         if(pUartControl->rxcnt != 0)
         {
            /* We removed data from the buffer, it is time to reorder it */
            for(i = 0; i < pUartControl->rxcnt ; i++)
            {
               pUartControl->hwbuf[i] = pUartControl->hwbuf[i + ret];
            }
         }
      }
   }
   return ret;
}

extern ssize_t ciaaDriverUart_write(ciaaDevices_deviceType const * const device, uint8_t const * const buffer, size_t const size)
{
   ssize_t ret = 0;

   if((device == ciaaDriverUartConst.devices[0]) ||
      (device == ciaaDriverUartConst.devices[1]) ||
      (device == ciaaDriverUartConst.devices[2]) )
   {
      while((Chip_UART_ReadLineStatus((LPC_USART_T *)device->loLayer) & UART_LSR_THRE) && (ret < size))
      {
         /* send first byte */
         Chip_UART_SendByte((LPC_USART_T *)device->loLayer, buffer[ret]);
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

   /* add uart driver to the list of devices */
   for(loopi = 0; loopi < ciaaDriverUartConst.countOfDevices; loopi++) {
      /* add each device */
      ciaaSerialDevices_addDriver(ciaaDriverUartConst.devices[loopi]);
   }
}

/*==================[interrupt handlers]=====================================*/
ISR(UART0_IRQHandler)
{
   uint8_t status = Chip_UART_ReadLineStatus(LPC_USART0);

   if(status & UART_LSR_RDR)
   {
      do
      {
         uartControl[0].hwbuf[uartControl[0].rxcnt] = Chip_UART_ReadByte(LPC_USART0);
         uartControl[0].rxcnt++;
      }while((Chip_UART_ReadLineStatus(LPC_USART0) & UART_LSR_RDR) &&
             (uartControl[0].rxcnt < UART_RX_FIFO_SIZE));

      ciaaDriverUart_rxIndication(&ciaaDriverUart_device0, uartControl[0].rxcnt);
   }
   if((status & UART_LSR_THRE) && (Chip_UART_GetIntsEnabled(LPC_USART0) & UART_IER_THREINT))
   {
      /* tx confirmation, 1 byte sent */
      ciaaDriverUart_txConfirmation(&ciaaDriverUart_device0);

      if(Chip_UART_ReadLineStatus(LPC_USART0) & UART_LSR_THRE)
      {  /* There is not more bytes to send, disable THRE irq */
         Chip_UART_IntDisable(LPC_USART0, UART_IER_THREINT);
      }
   }
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

