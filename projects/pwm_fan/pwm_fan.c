/*
 * File:   led_PWM_Pot_c.c
 * Author: gavin lyons
 * Url: https://github.com/gavinlyonsrepo/pic_12F675_projects
 * Created on 24 August 2018, 23:18
 * IDE: MPLAB X v4.2 
 * Compiler: xc8 v2.0
 */
 
/*  Name     : main.c
 *  Purpose  : Main file for software UART code for PIC12F675.
 *  Author   : M.Saeed Yasin
 *  Date     : 30-06-12
 *  Revision : None
 */

/*  Name            : pwm_fan.c
 *  Purpose         : pwm fan for Dreambox DM920
 *  URL             : https://github.com/munjeni/pic_12F675_projects/tree/master/projects/pwm_fan
 *  Code based on   : two authors work above this comment
 *  Date            : 29.04.2019
 *  Compiler        : xc8 v2.05
 *  IDE             : xc8 ide v5.15
 */

/*

========================= PIC12F675 PINOUT ============================

                                00
                            -----------
                    VDD  1 |          | 8  VSS
                           |          |
   GP5/T1CKI/OSC1/CLKIN  2 |          | 7  GP0/AN0/CIN+/ICSPDAT
                           |          |
GP4/AN3/T1G/OSC2/CLKOUT  3 |          | 6  GP1/AN1/CIN-/VREF/ICSPCLK
                           |          |
           GP3/MCLR/VPP  4 |          | 5  GP2/AN2/T0CKI/INT/COUT
                           ------------

=======================================================================

	ANSEL - ANALOG SELECT REGISTER
	------------------------------------------------------------
	 U-0   RW-0    RW-0    RW-0    RW-1   RW-1   RW-1   RW-1      (default values, 1=set, 0=set, x=unknown)
	  -    ADCS2   ADCS1   ADCS0   ANS3   ANS2   ANS1   ANS0
	 bit7   bit6    bit5    bit4   bit3   bit2   bit1   bit0
	------------------------------------------------------------
	
	bit 7 Unimplemented: Read as '0'.
	
	bit 6-4 ADCS<2:0>: A/D Conversion Clock Select bits
	000 = FOSC/2
	001 = FOSC/8
	010 = FOSC/32
	x11 = FRC (clock derived from a dedicated internal oscillator = 500 kHz max)
	100 = FOSC/4
	101 = FOSC/16
	110 = FOSC/64
	
	bit 3-0 ANS3:ANS0: Analog Select bits
	(Between analog or digital function on pins AN<3:0>, respectively.)
	1 = Analog input; pin is assigned as analog input(1)
	0 = Digital I/O; pin is assigned to port or special function

	Note 1: Setting a pin to an analog input automatically disables the digital input circuitry,
	weak pull-ups, and interrupt-on-change. The corresponding TRISIO bit must be set
	to Input mode in order to allow external control of the voltage on the pin.


	ADCON0 - A/D CONTROL REGISTER
	------------------------------------------------------------
	RW-0    RW-0    U-0     U-0    RW-0   RW-0    RW-0     RW-0     (default values, 1=set, 0=set, x=unknown)
	ADFM    VCFG     -       -     CHS1   CHS0   GO/DONE   ADON
	bit7    bit6   bit5    bit4    bit3   bit2    bit1     bit0
	------------------------------------------------------------

	bit 7 ADFM: A/D Result Formed Select bit
	1 = Right justified
	0 = Left justified

	bit 6 VCFG: Voltage Reference bit
	1 = VREF pin
	0 = VDD

	bit 5-4 Unimplemented: Read as zero

	bit 3-2 CHS1:CHS0: Analog Channel Select bits
	00 = Channel 00 (AN0)
	01 = Channel 01 (AN1)
	10 = Channel 02 (AN2)
	11 = Channel 03 (AN3)

	bit 1 GO/DONE: A/D Conversion Status bit
	1 = A/D conversion cycle in progress. Setting this bit starts an A/D conversion cycle.
	 This bit is automatically cleared by hardware when the A/D conversion has completed.
	0 = A/D conversion completed/not in progress

	bit 0 ADON: A/D Conversion STATUS bit
	1 = A/D converter module is operating
	0 = A/D converter is shut-off and consumes no operating current

	
	TRISIO - GPIO TRISTATE REGISTER
	-----------------------------------------------------------------
	 U-0     U-0    RW-x      RW-x     R-1        RW-x     RW-x     RW-x   (default values, 1=set, 0=set, x=unknown)
	  -       -    TRISIO5  TRISIO4  TRISIO3    TRISIO2  TRISIO1   TRISIO0
	 bit7    bit6   bit5      bit4    bit3        bit2     bit1     bit0
	-----------------------------------------------------------------

	bit 7-6: Unimplemented: Read as '0'
	
	bit 5-0: TRISIO<5:0>: General Purpose I/O Tri-State Control bit
	1 = GPIO pin configured as an input (tri-stated)
	0 = GPIO pin configured as an output.
	
	Note: TRISIO<3> always reads 1.


	OPTION_REG - OPTION REGISTER (ADDRESS: 81h)
	------------------------------------------------------------------
	RW-1    RW-1    RW-1     RW-1     RW-1     RW-1      RW-1     RW-1    ((default values, 1=set, 0=set)
	GPPU   INTEDG   T0CS     T0SE      PSA      PS2       PS1      PS0
	bit7    bit6    bit5     bit4     bit3     bit2      bit1     bit0
	-------------------------------------------------------------------

	bit 7 GPPU: GPIO Pull-up Enable bit
	1 = GPIO pull-ups are disabled
	0 = GPIO pull-ups are enabled by individual port latch values

	bit 6 INTEDG: Interrupt Edge Select bit
	1 = Interrupt on rising edge of GP2/INT pin
	0 = Interrupt on falling edge of GP2/INT pin

	bit 5 T0CS: TMR0 Clock Source Select bit
	1 = Transition on GP2/T0CKI pin
	0 = Internal instruction cycle clock (CLKOUT)

	bit 4 T0SE: TMR0 Source Edge Select bit
	1 = Increment on high-to-low transition on GP2/T0CKI pin
	0 = Increment on low-to-high transition on GP2/T0CKI pin

	bit 3 PSA: Prescaler Assignment bit
	1 = Prescaler is assigned to the WDT
	0 = Prescaler is assigned to the TIMER0 module

	bit 2-0 PS2:PS0: Prescaler Rate Select bits
	Bit Value        TMR0 Rate          WDT Rate
	   000              1/2               1/1
	   001              1/4               1/2
	   010              1/8               1/4
	   011              1/16              1/8
	   100              1/32              1/16
	   101              1/64              1/32
	   110              1/128             1/64
	   111              1/256             1/128
	
*/

// PIC12F675 Configuration Bit Settings
#pragma config FOSC = INTRCIO		// Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF		// Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON		// Power-Up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF		// MCLR
#pragma config BOREN = ON		// Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF			// Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF		// Data Code Protection bit (Data memory code protection is disabled)

#include <xc.h>
#include <stdio.h>	/* for sprintf */
#include <stdint.h>	/* For uint16_t definition */

// set 0 to disable debuging
#define ENABLE_UART_DEBUG   0
#define DEBUG_TO_EEPROM   0

// How many samples from LM35 for more accourate result
#define LM35_SAMPLES	40

// Define CPU Frequency
// This must be defined, if __delay_ms() or __delay_us() functions are used in the code
#define _XTAL_FREQ	4000000

/*
	PIC TIMER0 Calculator
	
	Clock Source in Mhz                   4 Mhz
	Fosc                                  4000000.0 Hz
	Fosc / 4                              1000000.0 Hz
	Time Period                           1e-06 sec
	Prescaler                             32
	Timer0 Interrupt Period               0.008192 sec
	Period of Frequency Input To Timer0   3.2e-05 sec
	Period of Time for each Timer0 Count  0.008192 sec
	(1000000/256) * 32 ~= 8mS				
*/

#if ENABLE_UART_DEBUG
#define Baudrate        1200                //bps
#define OneBitDelay     (1000000/Baudrate)  //...this might be wrong delay! See https://drive.google.com/file/d/0BwXmKaSw75eKdjI2elFsWW5Rd28/view
#define DataBitCount    8                   // no parity, no flow control
#define UART_TX         GP1                 // UART TX pin
#define UART_TX_DIR     TRISIO1             // UART TX pin direction register
#endif

#define PWM_Pin GP0
// Define PWM variable, It can have a value 
// from 0 (0% duty cycle) to 255 (100% duty cycle)
unsigned char PWM = 0;

#if ENABLE_UART_DEBUG
void UART_Transmit(unsigned char DataValue)
{
	unsigned char i;
	unsigned int count = 0;

	/* Basic Logic
	   
	   TX pin is usually high. A high to low bit is the starting bit and 
	   a low to high bit is the ending bit. No parity bit. No flow control.
	   BitCount is the number of bits to transmit. Data is transmitted LSB first.

	*/

	// Send Start Bit
	UART_TX = 0;
	//__delay_us(OneBitDelay);
	while(count < 8) {
		count++;
		__delay_us(83);
	}

	for (i=0; i < DataBitCount; i++)
	{
		// Set Data pin according to the DataValue
		if (((DataValue >> i) & 0x1) == 0x1)	//if Bit is high
		{
			UART_TX = 1;
		}
		else			// if Bit is low
		{
			UART_TX = 0;
		}

		//__delay_us(OneBitDelay);
		count = 0;
		while(count < 8) {
			count++;
			__delay_us(83);
		}
	}

	// Send Stop Bit
	UART_TX = 1;
	//__delay_us(OneBitDelay);
	count = 0;
	while(count < 8) {
		count++;
		__delay_us(83);
	}
}
#endif

unsigned int GetADCValue(void)
{
	//ADCON0 &= 0xf3;		// Clear Channel selection bits
	//ADCON0 |= 0x0c;		// Select GP4 pin as ADC input CHS1:CHS0: to 11

	__delay_ms(10);			// Time for Acqusition capacitor to charge up and show correct value

	GO_nDONE = 1;			// Enable Go/Done

	while(GO_nDONE);		//wait for conversion completion

	return ((ADRESH<<8)+ADRESL);	// Return 10 bit ADC value
}

void __interrupt() ISR(void)
{
	if(T0IF)			// If Timer0 Interrupt
	{
		if (!PWM)		// Since this frequency is in audible range, shutdown motor noise
		{
			PWM_Pin = 0;
			TMR0 = PWM;
		}
		else
		{
			if(PWM_Pin)	// if PWM_Pin is high
			{
				PWM_Pin = 0;
				TMR0 = PWM;
			}
			else		// if PWM_Pin is low
			{
				PWM_Pin = 1;
				TMR0 = 255 - PWM;
			}
		}

		T0IF = 0;		// Clear the interrupt
	}
}

void main(void)
{	
	unsigned int ADC_value = 0;
#if ENABLE_UART_DEBUG
	unsigned int temperature = 1;
	char str1[3];
	char str2[3];
	char str3[5];
#endif
	uint8_t i;

	OSCCAL = __osccal_val();    // Load Oscillator Calibration
	ANSEL  = 0x48;              // Clear Pin selection bits (set FOSC/4 ; ANS3 analog input)
	TRISIO = 0x10;              // GP4 input, rest all output
	GPIO   = 0x00;              // Clear gpio
	ADCON0 = 0x8D;              // Turn on the A/D Converter ADFM and ADON ; set channel to AN3
	CMCON  = 0x07;              // Shut off the Comparator, so that pins are available for ADC
	VRCON  = 0x00;	            // Shut off the Voltage Reference for Comparator

	// Initialize PWM
	// Use timer0 for making PWM
	OPTION_REG &= 0xC4;         // Intialise timer0 (prescaler 1/32 ; prescaler assigned to timer0)
	//INTCON = 0b10100000;      // Global Interrupt Enabled and TMR0 Overflow Interrupt Enabled
	TMR0 = 0;                   // Preload timer register
	T0IE = 1;                   // Enable Timer0 interrupt
	GIE = 1;                    // Enable global interrupts

#if ENABLE_UART_DEBUG
	// Intialize Soft UART
	UART_TX = 1;                // TX pin is high in idle state	GP1=1
	UART_TX_DIR = 0;            // set TRISIO1=0 as output direction
#endif

	while(1)
	{
		for (i=0; i < LM35_SAMPLES; i++)
			ADC_value += GetADCValue();

		ADC_value /= LM35_SAMPLES;

#if ENABLE_UART_DEBUG		
		for (i=0; i<5; i++) str3[i] = '\0';
#endif

		// Calculation formula: comparation_value = ( temperature_you_need * 0.01 ) / 0.004883
		// Temperature_you_need is temperature of the cpu heat sink and not a current cpu temperature!
		if (ADC_value < 76) PWM = 0;                        // lower than 37 C
		if (ADC_value >= 76 && ADC_value < 78) PWM = 77;    // lower than 38 C
		if (ADC_value >= 78 && ADC_value < 80) PWM = 102;   // lower than 39 C
		if (ADC_value >= 80 && ADC_value < 82) PWM = 127;   // lower than 40 C
		if (ADC_value >= 82 && ADC_value < 84) PWM = 152;   // lower than 41 C
		if (ADC_value >= 84 && ADC_value < 86) PWM = 177;   // lower than 42 C
		if (ADC_value >= 86 && ADC_value < 88) PWM = 202;   // lower than 43 C
		if (ADC_value >= 88 && ADC_value < 90) PWM = 227;   // lower than 44 C
		if (ADC_value >= 90 && ADC_value < 92) PWM = 245;   // lower than 45 C
		if (ADC_value >= 92) PWM = 255;                     // more  than 45 C

#if ENABLE_UART_DEBUG
		temperature = (ADC_value*49);

		sprintf(str1, "%u%u", (temperature/1000)%10, (temperature/100)%10);
		sprintf(str2, "%u%u", (temperature/10)%10, (temperature/1)%10);
		sprintf(str3, "%u", ADC_value);

		UART_Transmit('T');
		UART_Transmit('e');
		UART_Transmit('m');
		UART_Transmit('p');
		UART_Transmit('e');
		UART_Transmit('r');
		UART_Transmit('a');
		UART_Transmit('t');
		UART_Transmit('u');
		UART_Transmit('r');
		UART_Transmit('e');
		UART_Transmit(':');
		UART_Transmit(' ');
		for (i=0; i<2; i++) UART_Transmit(str1[i]);
		UART_Transmit('.');		
		for (i=0; i<2; i++) UART_Transmit(str2[i]);
		UART_Transmit(' ');
		UART_Transmit('C');

		UART_Transmit(',');
		UART_Transmit(' ');
		UART_Transmit('A');
		UART_Transmit('D');
		UART_Transmit('C');
		UART_Transmit(':');
		UART_Transmit(' ');
		for (i=0; i<5; i++)
		{
			if (str3[i] == '\0') break;
			UART_Transmit(str3[i]);
		}

		UART_Transmit(0x0D);
		UART_Transmit(0x0A);

#if DEBUG_TO_EEPROM
		/*  In case you can't get UART working so write log to EEPROM
		 *  e.g. T=18.62 celsius A=38 adc_value :)
		 */
		eeprom_write(0, 'T');
		eeprom_write(1, '=');
		eeprom_write(2, str1[0]);
		eeprom_write(3, str1[1]);
		eeprom_write(4, '.');
		eeprom_write(5, str2[0]);
		eeprom_write(6, str2[1]);
		eeprom_write(7, ' ');
		eeprom_write(8, 'A');
		eeprom_write(9, '=');
		for (i=10; i<15; i++)
		{
			if (str3[i-10] == '\0') break;
			eeprom_write(i, str3[i-10]);
		}
#endif
#endif

		__delay_ms(600);	// 1 second delay before next reading = 600 + (LM35_SAMPLES * 10) = 1000ms total
	}
}

