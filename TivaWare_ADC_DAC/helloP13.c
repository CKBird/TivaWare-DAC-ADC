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
volatile uint32_t ADCValuesPing[128];
volatile uint32_t ADCValuesPong[128];
volatile bool pong = false;
volatile int slope = 0;
volatile int oldSlope = 0;
volatile int ticks = 0;
volatile int ADCIndex = 0;
volatile uint32_t sum = 0;
volatile uint32_t average = 0;
volatile uint32_t onescomp = 0;
volatile uint16_t sendA = 0x0900;
volatile uint16_t sendB = 0x0A00;
volatile uint8_t aver = 0;
volatile uint8_t ones = 0;
int x = 0;
int count = 0;
double lows = 0.0;
bool found = false;
double freq = 0;

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
{ 
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	//Configure pins for SSI
	ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
	//Set to master mode (SPI), 8 bit data
	ROM_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 2000000, 16);

	//Enable SSI
	ROM_SSIEnable(SSI0_BASE);
}

void ConfigureTimer0ADC()
{
	//Timer Configure and enable
  	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
		ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
		ADCSequenceEnable(ADC0_BASE, 3);
		ADCIntClear(ADC0_BASE, 3);
		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  	ROM_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet())/30000); //Should give 10KHz
  	//ROM_IntEnable(INT_TIMER0A);
  	//ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/*void ConfigureADC()
{
}*/

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
	//UARTprintf("A");
	/* PART 1
	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceDataGet(ADC0_BASE, 3, &ADCValues[ADCIndex]);
	ADCIndex++;
	if(ADCIndex > 127) //Circular Index
		ADCIndex = 0;
	
	sum = ADCValues[ADCIndex - 1] + ADCValues[ADCIndex - 2] + ADCValues[ADCIndex - 3] + ADCValues[ADCIndex - 4];
	average = sum / 4;
	onescomp = ~average;
	*/
	
	ADCIntClear(ADC0_BASE, 3);
	if(ADCIndex == 128)
	{
		ADCIndex = 0;
		if(pong == true)
			pong = false;
		else
			pong = true;
	}
	if(pong)
	{
		ADCSequenceDataGet(ADC0_BASE, 3, &ADCValuesPong[ADCIndex]);
		sum = ADCValuesPong[ADCIndex - 1] + ADCValuesPong[ADCIndex - 2] + ADCValuesPong[ADCIndex - 3] + ADCValuesPong[ADCIndex - 4];
	}
	else
	{	
		ADCSequenceDataGet(ADC0_BASE, 3, &ADCValuesPing[ADCIndex]);
		sum = ADCValuesPing[ADCIndex - 1] + ADCValuesPing[ADCIndex - 2] + ADCValuesPing[ADCIndex - 3] + ADCValuesPing[ADCIndex - 4];
	}
	
	ADCIndex++;
	
	average = sum / 4;
	onescomp = ~average;
	
	aver = (average >> 4) & 0xFF;
	ones = (onescomp >> 4) & 0xFF;
	
	//aver = 0xFF;
	//ones = 0xFF;
	
	uint32_t dataA = sendA | aver;
	uint32_t dataB = sendB | ones;
		
 	SSIDataPut(SSI0_BASE, dataA);
 	while(SSIBusy(SSI0_BASE));

 	SSIDataPut(SSI0_BASE, dataB);
 	while(SSIBusy(SSI0_BASE));
	
}

void analyzeFunction(volatile uint32_t *used, volatile uint32_t *notUsed)
{
	int high = 0;
	int low = 10000;
	int count = 0;
	int peak = 0;
	bool lowFound = false;
	bool highFound = false;
	for(int i = 0; i < 128; i++)
	{
		if(used[i] > high)
			high = used[i];
		if(used[i] < low)
			low = used[i];
	} //Once complete, high and low are set with max/min values
	//UARTprintf("Low: %d --- High: %d ", low, high);
	for(int i = 0; i < 128; i++)
	{
		if(lowFound && (used[i] > (low+5)))
			count++;
		if(lowFound && used[i] <= (low+5))
			break;
		if(used[i] <= low)
			lowFound = true;
	} //Will fill count with number of samples between two lowest values
		//Used for frequency, will not work with square (value after first low is still low)
	UARTprintf("Count: %d\n", count);
	int final = 10000 / (2 * count);
	final *= 10;
	UARTprintf("Frequency: %d Hz\n", final); //Frequency in count * 10,000 Hz
	for(int i = 0; i < 128; i++)
	{
		if(used[i] >= (high-30))
			peak++;
		if(peak > 0 && used[i] < (high-30))
			break;
	}
	//UARTprintf("Peak: %d", peak);
	if(peak > 8)
		UARTprintf("SQUARE");
	else if(peak > 2)
		UARTprintf("SIN");
	else
		UARTprintf("TRIANGLE");
	UARTprintf("\n\n");
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
	ConfigureSwitches();				//Configure Switches
	ConfigureTimer0ADC(); 					//Configure Timer
	//ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);    
	//ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
  //ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	//ROM_IntEnable(INT_UART1);
  //ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);   
	ROM_ADCIntEnable(ADC0_BASE, 3);
	ROM_IntEnable(INT_ADC0SS3);  	
	
	ROM_IntMasterEnable();
	
	//int sw_input;
	
 	while(1)
 	{	
		if(ticks % 200000 == 0)
		{
			if(!pong)
				analyzeFunction(ADCValuesPing, ADCValuesPong);
			else
				analyzeFunction(ADCValuesPong, ADCValuesPing);
		}

		if((ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & 0x10) != 0x10)
		{
			/*for(int i = 0; i < 127; i++)
			{
				if(i % 8 == 0)
					UARTprintf("\n");
				UARTprintf("%d ", ADCValues[i]);
			}*/
			UARTprintf("Ping: \n");
			for(int i = 0; i < 128; i++)
			{
				if(i % 8 == 0)
					UARTprintf("\n");
				UARTprintf("%d ", ADCValuesPing[i]);
			}
			UARTprintf("Pong: \n");
			for(int i = 0; i < 128; i++)
			{
				if(i % 8 == 0)
					UARTprintf("\n");
				UARTprintf("%d ", ADCValuesPong[i]);
			}
		UARTprintf("Average of Last 4: %d \nOnes Complement of Average: %d", average, onescomp);
		}
		/*if (!ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) 
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
		else
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);*/
 	ticks++;
	}
}
