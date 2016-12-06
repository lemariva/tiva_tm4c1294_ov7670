//*****************************************************************************
//	Copyright [2016] [Mauro Hernan Riva (lemariva)]
//
//	Licensed under the Apache License, Version 2.0 (the "License");
//	you may not use this file except in compliance with the License.
//	You may obtain a copy of the License at
//
//		http://www.apache.org/licenses/LICENSE-2.0
//
//	Unless required by applicable law or agreed to in writing, software
//	distributed under the License is distributed on an "AS IS" BASIS,
//	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//	See the License for the specific language governing permissions and
//	limitations under the License.
//***********************************************************************

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"

#include "driverlib/pwm.h"
//#include "utils/uartstdio.h"

#include "driverlib/uart.h"
#include "driverlib/i2c.h"


#include "utils.h"
#include "pins.h"
#include "tprintf.h"
#include "ov7670.h"

void ConfigureBoard(void);
void UARTIntHandler(void);

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

static volatile uint8_t isProcessing = 0; // Status for when the device is processing
static volatile uint8_t hasReceived = 0; // Lets the program know when a byte is received
static volatile uint8_t rcvbuf[16]; // Value recieved once hasRecieved is set
static volatile uint8_t led_status = 1;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


void ConfigureBoard(void)
{
    // Enable and configure the GPIO port
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA | SYSCTL_PERIPH_GPIOB | SYSCTL_PERIPH_GPIOC | SYSCTL_PERIPH_GPIOD | SYSCTL_PERIPH_GPIOE | SYSCTL_PERIPH_GPIOF);
	//ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

	//ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    // PORT A
    ROM_GPIOPinTypeGPIOOutput(RRST_PORT, RRST);
    ROM_GPIOPinWrite(RRST_PORT, RRST, 0x00);
    GPIOPadConfigSet(RRST_PORT,  RRST, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    // PORT P
    ROM_GPIOPinTypeGPIOInput(VSYNC_PORT, VSYNC);
    GPIOPadConfigSet(VSYNC_PORT,  VSYNC, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    ROM_GPIOPinTypeGPIOOutput(RCLK_PORT, RCLK);
    ROM_GPIOPinWrite(RCLK_PORT, RCLK, 0x00);
    GPIOPadConfigSet(RCLK_PORT,  RCLK, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    // PORT M
    ROM_GPIOPinTypeGPIOOutput(WEN_PORT, WEN);
    ROM_GPIOPinWrite(WEN_PORT, WEN, 0x00);
    GPIOPadConfigSet(WEN_PORT, WEN, GPIO_STRENGTH_12MA , GPIO_PIN_TYPE_STD);

    // PORT K
    ROM_GPIOPinTypeGPIOInput(DATA_PORT, GPIO_DATA);
    GPIOPadConfigSet(DATA_PORT, GPIO_DATA, GPIO_STRENGTH_12MA , GPIO_PIN_TYPE_STD);

    // PORT N
    //GPIOPinTypeGPIOOutput(LEDS_PORT, RED_LED|GREEN_LED);
    //GPIOPinWrite(LEDS_PORT, RED_LED|GREEN_LED, 0x00);
    //GPIOPadConfigSet(LEDS_PORT,  RED_LED|GREEN_LED, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}


void ConfigurePWM(void)
{
//	unsigned long ulPeriod;

	//Configure PWM Clock to match system
//	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //Enable the peripherals used by this program.
//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

//	ulPeriod = SysCtlClockGet() / 1200; //PWM frequency
	//Configure PWM Options
//	GPIOPinConfigure(GPIO_PF2_M1PWM6);
//	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

	//Configure PWM Options
	//PWM_GEN_2 Covers M1PWM4 and M1PWM5
	//PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
//	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	//Set the Period (expressed in clock ticks)
//	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ulPeriod);

	//Set PWM duty-50% (Period /2)
//	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ulPeriod/2);

	// Enable the PWM generator
//	PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the Output pins
//    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
}
//*****************************************************************************
//
// Configure the UART and its pins for the camera
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable UART6
    //
	ROM_SysCtlPeripheralEnable(PERIPH_UART);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_UARTRX);
    ROM_GPIOPinConfigure(GPIO_UARTTX);
    ROM_GPIOPinTypeUART(PORT_UART,  TXD | RXD);

    //
    // Use oscillator as the UART clock source.
    //
    ROM_UARTConfigSetExpClk		(UART_UNIT, g_ui32SysClock, 115200,	//115200 internal 16mHz oscilator -> UART_CLOCK_PIOSC (UARTClockSourceSet(UART_UNIT, UART_CLOCK_PIOSC))
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    //
    // Initialize the UART interruption
    //
    // Set both the TX and RX trigger thresholds to 4.  This will be used by
	// the uDMA controller to signal when more data should be transferred.  The
	// uDMA TX and RX channels will be configured so that it can transfer 2
	// bytes in a burst when the UART is ready to transfer more data.
	//
    ROM_UARTFIFOLevelSet(UART_UNIT, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    UARTIntRegister(UART_UNIT, UARTIntHandler);

    ROM_IntEnable(UART_INT);
    ROM_UARTIntEnable(UART_UNIT, UART_INT_RX | UART_INT_RT);

}


uint8_t uart_received()
{
    if (!hasReceived) {
        return 0;
    }

    hasReceived = 0;
    return 1;
}

//*****************************************************************************
//
// Configure the I2C and its pins for ov7670
//
//*****************************************************************************
void
ConfigureI2C(void)
{
	//
	// Enable I2C2
	//
	//
	ROM_SysCtlPeripheralEnable(PERIPH_I2C);
	ROM_SysCtlPeripheralEnable(PER_PORT_I2C);

    //
    // Configure GPIO Pins for I2C mode.
    //
	GPIOPinTypeI2CSCL(PORT_I2C, SCL);
	ROM_GPIOPinTypeI2C(PORT_I2C, SDA);


	ROM_GPIOPinConfigure(GPIO_I2CSCL);
	ROM_GPIOPinConfigure(GPIO_I2CSDA);


	// Use oscillator as the I2C clock source.
	// Initialize I2C2 peripheral.

	ROM_I2CMasterInitExpClk(I2C_UNIT, g_ui32SysClock, false);  //false = 100 Khz, true = 400KHz m_I2CBus

	ROM_SysCtlDelay(10000);  // delay mandatory here - otherwise portion of SlaveAddrSet() lost!

	ROM_I2CMasterEnable(I2C_UNIT);

}
//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)
{
	uint8_t i;
    //
    // Make sure the main oscillator is enabled because this is required by
    // the PHY.  The system must have a 25MHz crystal attached to the OSC
    // pins.  The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
    // frequency is 10MHz or higher.
    //
    ROM_SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);


    // Setup the system clock to run at 50 Mhz from PLL with crystal reference
    //SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Set the clocking to run directly from the crystal at 120MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                							SYSCTL_OSC_MAIN |
                							SYSCTL_USE_PLL |
                							SYSCTL_CFG_VCO_480), 120000000);

    // Port/Pin configuration
    ConfigureBoard();

    // UART configuration
    ConfigureUART();

    // PWM Enable
    //ConfigurePWM();
    // enable interruptions
    ROM_IntMasterEnable();

	// ******************************************
    // Welcome message
    tprintf("Hello world!\r\n");

    wait(delay_120ns);
    tprintf("Initializing i2c...");

    // I2C configuration
    ConfigureI2C();

    wait(delay_120ns);
    tprintf("done.\r\n");

    tprintf("Initializing ov7670...\r\n");
    for (i = 0; i < 10; i ++) {
    	bool initialized = ov7670_init();
        if (initialized) {
            break;
        } else if (i == 5) {
            tprintf("\r\nPANIC! ov7670 init keeps failing!\r\n");
            while (1);
        } else {
            tprintf("retrying...\r\n");
            wait(delay_120ns);
        }
    }
    wait(delay_120ns);
    tprintf("done.\r\n");

    tprintf("READY\r\n");

    while(1)
    {

    }
}


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    uint8_t arrayptr;
    uint8_t tmp_char;
    uint16_t j = 0;
    // Get the interrupt status.
    ui32Status = ROM_UARTIntStatus(UART_UNIT, true);
    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART_UNIT, ui32Status);

    arrayptr = 0;
    tmp_char = 0;
    // Loop while there are characters in the receive FIFO.
	while(ROM_UARTCharsAvail(UART_UNIT) && tmp_char != '\r')
	{
		// Read the next character from the UART and write it back to the UART.
		//UARTCharPutNonBlocking(UART_UNIT, UARTCharGetNonBlocking(UART_UNIT));
		tmp_char = ROM_UARTCharGetNonBlocking(UART_UNIT);
		if ((tmp_char >= 32) && (tmp_char <= 126)) {
			rcvbuf[arrayptr++] = tmp_char;
		} else if (tmp_char == 13) {
			// Turn on the LED
			led_status ^= 1;
			//if(led_status==1)
				//GPIOPinWrite(LEDS_PORT, RED_LED, RED_LED);
			//else
				//GPIOPinWrite(LEDS_PORT, RED_LED, ~RED_LED);
			hasReceived = 1;
			break;
		}
		ROM_SysCtlDelay(8000);
	}

	if(hasReceived==1)
		{
			hasReceived = 0;
			if (strcmp((char *) rcvbuf, "hello") == 0) {
				tprintf("Hello to you too!\r\n");
			} else if (strncmp((char *) rcvbuf, "cap", 3) == 0) {
				ov7670_capture();
				tprintf("OK\r\n");
			} else if (strncmp((char *) rcvbuf, "rrst", 4) == 0) {
				ov7670_rrst();
				tprintf("OK\r\n");
			} else if (strncmp((char *) rcvbuf, "init", 4) == 0) {
				ov7670_init();
				tprintf("OK\r\n");
			} else if (strncmp((char *) rcvbuf, "img", 3) == 0) {
					ov7670_image();
			} else if (strlen((char *) rcvbuf) > 5 &&
					strncmp((char *) rcvbuf, "read ", 5) == 0) {
				for (j = 0; j < atoi((char *) (rcvbuf + 5)); j++) {
					ROM_UARTCharPut(UART_UNIT, ov7670_read());
				}
	        } else if (strlen((char *) rcvbuf) > 6 &&
	                strncmp((char *) rcvbuf, "hread ", 6) == 0) {
	            for (j = 0; j < atoi((char *) (rcvbuf + 6)); j++) {
	            	tprintf("Data: [0x%x] j: %i\r\n", ov7670_read(), j);
	            }
	        }
		}


	    //IntEnable(UART1_BASE);
	}
