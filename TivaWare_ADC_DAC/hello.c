#include "stdint.h"
#include "stdbool.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "stdio.h"
#include <string.h>
#include "stdlib.h"
#include "math.h"
//#include "Adafruit_GFX.h"
//#include "glcdfont.c"

volatile uint32_t ADCValues[128];
volatile int ADCIndex = 0;

#define sendA 253
#define sendB 254

void ConfigureUART0(void) //Config local UART for Teraterm/Console use
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //UART Pin A
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    ROM_GPIOPinConfigure(GPIO_PA0_U0RX); //Pin MUX (In/Out)
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 9600, 16000000);
}

/*void ConfigureUART1(void) //Configure UART for Tx/Rx functionality
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //UART Pin B
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    ROM_GPIOPinConfigure(GPIO_PB0_U1RX); //Pin MUX (In/Out)
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    ROM_UARTConfigSetExpClk(UART1_BASE, 16000000, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}*/

void ConfigureSSI (void) 
{ //Configure SSI for OLED
	//Configure muxs to enable special pin functions
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	//GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	//Configure pins for SSI
	ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2);
	//Set to master mode (SPI), 8 bit data
	ROM_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	UARTprintf("OEU");

	//Enable SSI
	ROM_SSIEnable(SSI0_BASE);
}

void ConfigureTimer0()
{
	//Timer Configure and enable
  	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  	ROM_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()-1)/10000); //Should give 10KHz
  	//ROM_IntEnable(INT_TIMER0A);
  	//ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void ConfigureADC()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntClear(ADC0_BASE, 3);
}

void ConfigureSwitches()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
}

/*void Timer0INT_Handler()
{
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

}*/

void ADCINT_Handler ()
{
	UARTprintf("A");
	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceDataGet(ADC0_BASE, 3, &ADCValues[ADCIndex]);
	ADCIndex++;
	if(ADCIndex > 127) //Circular Index
		ADCIndex = 0;
}

int main (void) {
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); 								//B
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);        	//pinMode(_rst, OUTPUT);
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
  //ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_7 );  				//Configure pin as input
      
  //Configure chosen pin for interrupts
  //ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_BOTH_EDGES); //Interrupt triggered on both edges (rising/falling distinguished in interrupt handler)

  //Enable interrupts (on pin, port, and master)
  //GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_7);
  //ROM_IntEnable(INT_GPIOB); 
  
	ConfigureUART0(); 					//Config Standard Console
	UARTprintf("Turning on now\n");
  //ConfigureUART1(); 				//Configure Rx/Tx
	ConfigureSSI(); 						//Configure DAC
		UARTprintf("OEU");

	ConfigureTimer0(); 					//Configure Timer
	ConfigureADC();							//Configure ADC
	ConfigureSwitches();				//Configure Switches
	//ROM_IntEnable(INT_UART1);
  //ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);   
	ROM_IntEnable(INT_ADC0SS3);
	ROM_ADCIntEnable(ADC0_BASE, 3);
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);  	
  
	ROM_IntMasterEnable();

	int sw_input;
 	while(1)
 	{
 		do {
 			UARTprintf("1");
			sw_input = ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
 		} while ((sw_input & 0x10) == 0x10);

		UARTprintf("CONFUSION");
		
 		uint32_t sum = ADCValues[ADCIndex - 1] + ADCValues[ADCIndex - 2] + ADCValues[ADCIndex - 3] + ADCValues[ADCIndex - 4];
 		uint32_t average = sum / 4;
 		uint32_t onescomp = ~average;

 		for(int i = 0; i < 127; i++)
 		{
 			if(i % 8 == 0)
 				UARTprintf("\n");
 			UARTprintf("%d", ADCValues[i]);
 		}

 		UARTprintf("Average of Last 4: %d \nOnes Complement of Average: %d", average, onescomp);

 		SSIDataPut(SSI0_BASE, sendA); //These 4 may not work...
 		while(SSIBusy(SSI0_BASE));

 		SSIDataPut(SSI0_BASE, average);
 		while(SSIBusy(SSI0_BASE));

 		SSIDataPut(SSI0_BASE, sendB);
 		while(SSIBusy(SSI0_BASE));

 		SSIDataPut(SSI0_BASE, onescomp);
 		while(SSIBusy(SSI0_BASE));
 	}
}
