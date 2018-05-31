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

/*
const Port Ports[NUM_PORTS]=
{
  { .pGPIO=PIOA, .ulId=ID_PIOA },
  { .pGPIO=PIOB, .ulId=ID_PIOB }
};
*/

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
	//	pPort	Pin			PeriphId	PinType			PinConfig		PinAttribute		ADCCh	PWMCh		TCCh
	{	PIOA,	PIO_PA0,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA1,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA2,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA3,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //TWD3 (SDA)
	{	PIOA,	PIO_PA4,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //TWCK3 (SCL)
	{	PIOA,	PIO_PA5,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA6,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA7,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA8,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA9,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA10,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA11,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA12,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI5_MISO
	{	PIOA,	PIO_PA13,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI5_MOSI
	{	PIOA,	PIO_PA14,	ID_PIOA,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //SPI5_SCK
	{	PIOA,	PIO_PA15,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA16,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA17,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC0,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA18,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC1,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA19,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC2,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA20,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC3,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA21,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA22,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA23,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA24,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA25,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA26,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA27,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA28,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA29,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA30,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},
	{	PIOA,	PIO_PA31,	ID_PIOA,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	},

	{	PIOB,	PIO_PB0,	ID_PIOB,	PIO_PERIPH_A,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC4,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D32
	{	PIOB,	PIO_PB1,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC5,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D33
	{	PIOB,	PIO_PB2,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC6,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D34
	{	PIOB,	PIO_PB3,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_ANALOG,	ADC7,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D35
	{	PIOB,	PIO_PB4,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D36
	{	PIOB,	PIO_PB5,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D37
	{	PIOB,	PIO_PB6,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D38
	{	PIOB,	PIO_PB7,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D39
	{	PIOB,	PIO_PB8,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D40
	{	PIOB,	PIO_PB9,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D41
	{	PIOB,	PIO_PB10B_TXD6,	ID_PIOB,	PIO_PERIPH_B,	PIO_DEFAULT,	(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D42
	{	PIOB,	PIO_PB11B_RXD6,	ID_PIOB,	PIO_PERIPH_B,	PIO_DEFAULT,	(PIN_ATTR_DIGITAL|PIN_ATTR_COMBO),	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D43
	{	PIOB,	PIO_PB12,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D44
	{	PIOB,	PIO_PB13,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D45
	{	PIOB,	PIO_PB14,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}, //D46
	{	PIOB,	PIO_PB15,	ID_PIOB,	PIO_OUTPUT_0,	PIO_DEFAULT,	PIN_ATTR_DIGITAL,	NO_ADC,	NOT_ON_PWM,	NOT_ON_TIMER	}  //D47
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
	__libc_init_array();

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
