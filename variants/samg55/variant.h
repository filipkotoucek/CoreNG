/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_SAMG55_H
#define _VARIANT_SAMG55_H

#ifndef __SAMG55J19__
#error Wrong variant.h file included!
#endif

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			120000000

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Core.h"

#ifdef __cplusplus
//#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define APINS_COUNT			(48u)
static const uint32_t MaxPinNumber = 103;	// last GPIO pin

/*
 * SPI Interfaces
 */

#define APIN_SPI_INTERFACE_ID	ID_FLEXCOM7	//ID_SPI
#define APIN_SPI_MOSI		(28u)
#define APIN_SPI_MISO		(27u)
#define APIN_SPI_SCK		(29u)
#define APIN_SPI_SS0		(30u)
#define APIN_SPI_SS1		(31u)	//SHIFTREG

#define APIN_USART0_MOSI	(27u)
#define APIN_USART0_MISO	(26u)
#define APIN_USART0_SCK		(30u)

#define APIN_USART1_MOSI	(22u)
#define APIN_USART1_MISO	(21u)
#define APIN_USART1_SCK		(23u)

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define APIN_WIRE_SDA		(PIO_PA3_IDX)	//(3u)
#define APIN_WIRE_SCL		(PIO_PA4_IDX)	//(4u)
#define FLEXCOM_INTERFACE   FLEXCOM3
#define WIRE_INTERFACE		TWI3	//TWI0
#define WIRE_INTERFACE_ID	ID_FLEXCOM3	//ID_TWI0
#define WIRE_ISR_HANDLER	FLEXCOM3_Handler //TWI0_Handler
#define WIRE_ISR_ID			FLEXCOM3_IRQn //TWI0_IRQn

/*
 * UART/USART Interfaces
 */
// SerialUSB
#define USB_VBUS_PIN		(54u)
// Serial
#define APINS_UART0			(107u)
#define APIN_UART0_RXD		(9u)
#define APIN_UART0_TXD		(10u)
// Serial1
#define APINS_UART1			(108u)
#define APIN_UART1_RXD		(5u)
#define APIN_UART1_TXD		(6u)

/*
 * Duet NG pins
 */
/*
// HSMCI
static const uint8_t APIN_HSMCI_CLOCK = 104;
static const uint8_t APINS_HSMCI_DATA = 105;

static const uint32_t MaxPinNumber = 103;						// last GPIO pin

static const uint32_t PwmFastClock = 25000 * 255;				// fast PWM clock for Intel spec PWM fans that need 25kHz PWM
static const uint32_t PwmSlowClock = (25000 * 255) / 256;		// slow PWM clock to allow us to get slow speeds
*/

/*
 * SAMG55 Einsy specific settings
 */

//frequency of all pwm outputs. 16b timer is used in mode WAVESEL=2 with input clock 120 MHz.
#define PWM_FREQ			20000



#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern USARTClass Serial1;
extern void ConfigurePin(const PinDescription& pinDesc);

#endif

#endif /* _VARIANT_SAMG55_H */
