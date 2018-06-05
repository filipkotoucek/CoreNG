/*
  Copyright (c) 2015 Thibaut VIARD.  All right reserved.

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

#include "variant.h"

//#ifdef __cplusplus
//extern "C" {
//#endif

/* TODO: change hw mapping system to something more abstract and usable
typedef enum {
	PWM_E0 = 0,			//PA0
	PWM_FAN0,			//PA1
	POWER_PANIC,		//PA2
	UART_0_TX,			//PA3
	UART_0_RX,			//PA4
	UART_1_RX,			//PA5
	UART_1_TX,			//PA6
	crystal_in,			//PA7
	crystal_out,		//PA8
	I2C_SCK,			//PA9
	I2C_SDA,			//PA10
	SPI1_CS0,			//PA11 (SD card)
	SPI1_MISO,			//PA12
	SPI1_MOSI,			//PA13
	SPI1_CLK,			//PA14
	TACH_0,				//PA15
						//PA16
	AD_E0,				//PA17
	AD_BED,				//PA18
	AD_PINDA0,			//PA19
						//PA20
	USB_DM,				//PA21
	USB_DP,				//PA22
						//PA23
						//PA24
	PINDA0_DETECT,		//PA25
	TACH_1,				//PA26
	SPI0_MISO,			//PA27
	SPI0_MOSI,			//PA28
	SPI0_CLK,			//PA29
	SPI0_CS0,			//PA30 (TMC2130)
	SPI0_CS1,			//PA31 (16b shift register)
						//PB0
						//PB1
	SPI1_CS1,			//PB2
						//PB3
	SWD_TDI,			//PB4
	SWD_TDO,			//PB5
	SWDIO,				//PB6
	SWCLK,				//PB7
	DIAG_0,				//PB8
	DIAG_1,				//PB9
	STEP_X,				//PB10
	STEP_Y,				//PB11
	STEP_Z,				//PB12
	STEP_E0,			//PB13
						//PB14
						//PB15
	TOTAL_USED_PINS
} PinHumanName;
*/

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
	//	pPort	Pin			PeriphId	PinType			PinConfig		PinAttribute		ADCCh	PWMCh		TCCh

	//pin 0-1
	{	PIOA,	PIO_PA0,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	PWM_CH0,	NOT_ON_TIMER	}, //PWM_E0
	{	PIOA,	PIO_PA1,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	PWM_CH3,	NOT_ON_TIMER	}, //PWM_FAN0

	//pin 2
	{	PIOA,	PIO_PA2,	ID_PIOA,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //POWER_PANIC

	//pin 3-4
	{	PIOA,	PIO_PA3,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //UART_0_TX
	{	PIOA,	PIO_PA4,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //UART_0_TX

	//pin 5-6
	{	PIOA,	PIO_PA5,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //UART_1_RX
	{	PIOA,	PIO_PA6,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //UART_1_TX

	//pin 7-8 TODO: configure for crystal
	{	PIOA,	PIO_PA7,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //crystal
	{	PIOA,	PIO_PA8,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //crystal

	//pin 9-10
	{	PIOA,	PIO_PA9,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //I2C_SCK
	{	PIOA,	PIO_PA10,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //I2C_SDA

	//pin 11-14
	{	PIOA,	PIO_PA11,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI1_CS0 (SD card)
	{	PIOA,	PIO_PA12,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI1_MISO
	{	PIOA,	PIO_PA13,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI1_MOSI
	{	PIOA,	PIO_PA14,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI1_CLK

	//pin 15-20
	{	PIOA,	PIO_PA15,	ID_PIOA,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //TACH_0 (cool fan)
	{	PIOA,	PIO_PA16,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused
	{	PIOA,	PIO_PA17,	ID_PIOA,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC0,	NOT_ON_PWM,	NOT_ON_TIMER	}, //AD_E0
	{	PIOA,	PIO_PA18,	ID_PIOA,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC1,	NOT_ON_PWM,	NOT_ON_TIMER	}, //AD_BED
	{	PIOA,	PIO_PA19,	ID_PIOA,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC2,	NOT_ON_PWM,	NOT_ON_TIMER	}, //AD_PINDA0
	{	PIOA,	PIO_PA20,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused

	//pin 21-22  TODO: configure for USB
	{	PIOA,	PIO_PA21,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //USB_DM
	{	PIOA,	PIO_PA22,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //USB_DP

	//pin 23-26
	{	PIOA,	PIO_PA23,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused
	{	PIOA,	PIO_PA24,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused
	{	PIOA,	PIO_PA25,	ID_PIOA,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //PINDA0_DETECT
	{	PIOA,	PIO_PA26,	ID_PIOA,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //TACH_1

	//pin 27-31
	{	PIOA,	PIO_PA27,	ID_PIOA,	PIO_PERIPH_B,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI0_MISO
	{	PIOA,	PIO_PA28,	ID_PIOA,	PIO_PERIPH_B,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI0_MOSI
	{	PIOA,	PIO_PA29,	ID_PIOA,	PIO_PERIPH_B,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI0_CLK
	{	PIOA,	PIO_PA30,	ID_PIOA,	PIO_PERIPH_B,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI0_CS0 (TMC2130)
	{	PIOA,	PIO_PA31,	ID_PIOA,	PIO_PERIPH_B,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI0_CS1 (16b shift register)

	//pin 32-35
	{	PIOB,	PIO_PB0,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused
	{	PIOB,	PIO_PB1,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused
	{	PIOB,	PIO_PB2,	ID_PIOB,	PIO_PERIPH_B,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI1_CS1
	{	PIOB,	PIO_PB3,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused

	//pin 36-39
	{	PIOB,	PIO_PB4,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SWD_TDI
	{	PIOB,	PIO_PB5,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SWD_TDO
	{	PIOB,	PIO_PB6,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SDWIO
	{	PIOB,	PIO_PB7,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SWCLK

	//pin 40-45
	{	PIOB,	PIO_PB8,	ID_PIOB,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //DIAG0
	{	PIOB,	PIO_PB9,	ID_PIOB,	PIO_INPUT,		PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //DIAG1
	{	PIOB,	PIO_PB10,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //STEP_X
	{	PIOB,	PIO_PB11,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //STEP_Y
	{	PIOB,	PIO_PB12,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //STEP_Z
	{	PIOB,	PIO_PB13,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //STEP_E0

	//pin 46-47
	{	PIOB,	PIO_PB14,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //unused
	{	PIOB,	PIO_PB15,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}  //unused


	//expansion IO by shift registers
	//pin 48-52
	//ENABLE pins for stepper

	//pin 53-57
	//DIR pins for stepper
};

//#ifdef __cplusplus
//}
//#endif

/*
 * UART objects
 */
RingBuffer rx_buffer2;
RingBuffer tx_buffer2;
USARTClass Serial1(USART3, FLEXCOM3_IRQn, ID_FLEXCOM3, &rx_buffer2, &tx_buffer2);

void FLEXCOM6_Handler(void)
{
  Serial1.IrqHandler();
}
/*
RingBuffer rx_buffer1;
RingBuffer tx_buffer1;
UARTClass Serial(UART0, UART0_IRQn, ID_UART0, &rx_buffer1, &tx_buffer1);
void UART0_Handler(void)
{
  Serial.IrqHandler();
}
*/
// ----------------------------------------------------------------------------

extern "C" void __libc_init_array(void);
extern void UrgentInit();

void ConfigurePin(const PinDescription& pinDesc)
{
	pio_configure(pinDesc.pPort, pinDesc.ulPinType, pinDesc.ulPin, pinDesc.ulPinConfiguration);
}

extern "C" void init( void )
{
	SystemInit();

	// Set Systick to 1ms interval, common to all SAM4 variants
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		// Capture error
		while (true);
	}

	UrgentInit();			// initialise anything in the main application that can't wait

	// Initialize C library (I think this calls C++ constructors for static data too)
	//__libc_init_array();

	// We no longer disable pullups on all pins here, better to leave them enabled until the port is initialised

	// Initialize Serial port U(S)ART pins
	ConfigurePin(g_APinDescription[APINS_UART0]);
	setPullup(APIN_UART0_RXD, true); 							// Enable pullup for RX0

	// No need to initialize the USB pins on the SAM4E because they are USB by default

	// Initialize Analog Controller
	AnalogInInit();

	// Initialize analogOutput module
	AnalogOutInit();

	// Initialize HSMCI pins
	//ConfigurePin(g_APinDescription[APIN_HSMCI_CLOCK]);
	//ConfigurePin(g_APinDescription[APINS_HSMCI_DATA]);
}
