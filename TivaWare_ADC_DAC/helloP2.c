#include "stdint.h"
#include "stdbool.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"

#include "stdio.h"
#include <string.h>
#include "stdlib.h"
#include "math.h"

#define PI 3.14159265
#define SENDA 0x09

//100 
volatile int8_t sinValues[128];
volatile int sinIndex = 0;
volatile uint8_t sendData[2];
	
//uDMA Control Table
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

void Timer0INT_Handler()
{
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	
	//Create a 16 bit(2 byte) block of memory for DMA to grab and put into SSI register
	sendData[0] = SENDA;
	sendData[1] = sinValues[sinIndex];
	
	ROM_uDMAChannelTransferSet(UDMA_CHANNEL_TMR0A | UDMA_PRI_SELECT,
                                UDMA_MODE_BASIC, 
                                sendData, (void *)(SSI0_BASE + SSI_O_DR),
                                sizeof(sendData));
																
	uDMAChannelEnable(UDMA_CH18_TIMER0A);
}
	
void ConfigureuDMA(void) {
    //Enable DMA Controller
    uDMAEnable();
    uDMAControlBaseSet(&ui8ControlTable[0]);
	
		uDMAChannelAssign(UDMA_CH18_TIMER0A);
    
    //Enable/Disable attributes
    //Disabled by default
    
    //Setup transfer characteristics
    uDMAChannelControlSet(UDMA_CHANNEL_TMR0A | UDMA_PRI_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE |
                              UDMA_ARB_1); 
		uDMAChannelEnable(UDMA_CH18_TIMER0A);
}

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

void ConfigureTimer0()
{
	//Timer & ADC Configure
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	//ROM_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet())/30000); //Should give 10KHz
    
	
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_DMA);
  TimerIntEnable(TIMER0_BASE,TIMER_TIMA_DMA);
  
	
    //Set up Timer to trigger DMA interrupts
  TimerDMAEventSet(TIMER0_BASE, TIMER_DMA_TIMEOUT_A );
  	//ROM_IntEnable(INT_TIMER0A);
  	//ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

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
    
    //Set up SSI Tx to receive directly from DMA
  ROM_SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);

	//Enable SSI
	ROM_SSIEnable(SSI0_BASE);
}

void createWave(void) {
	//Record sine value at each 128th of one period (2*PI)
	for (int i=0; i<128; i++) {
		sinValues[i] = 2000*sin(2*PI*(i/128));
	}
}

int main (void) {
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  
	ConfigureUART0(); 					//Config Standard Console
	UARTprintf("Turning on now\n");
	ConfigureSSI(); 						//Configure DAC
	ConfigureTimer0(); 					//Configure Timer   
	
	//Create wave buffer
	createWave();
	
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	ROM_IntMasterEnable();

	
 	while(1)
 	{	
		ROM_SysCtlSleep();
	}
}
