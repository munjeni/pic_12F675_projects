/*  Name            : ir_switch.c
 *  Purpose         : infrared simple switch
 *  URL             : https://github.com/munjeni/pic_12F675_projects/tree/master/projects/ir_switch
 *  Author          : munjeni
 *  Date            : 24.06.2019
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
          U-0   RW-0    RW-0    RW-0   RW-1   RW-1   RW-1   RW-1      (default values, 1=set, 0=set, x=unknown)
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
	bit7     bit6   bit5      bit4    bit3        bit2     bit1     bit0
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
#pragma config FOSC = INTRCIO	// Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF		// Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF		// Power-Up Timer Disable bit (PWRT disabled)
#pragma config MCLRE = OFF		// MCLR
#pragma config BOREN = OFF		// Brown-out Detect Enable bit (BOD disabled)
#pragma config CP = OFF			// Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF		// Data Code Protection bit (Data memory code protection is disabled)

#include <xc.h>
#include <stdio.h>	/* for sprintf */
#include <stdint.h>	/* For uint16_t definition */

// set 0 to disable debuging
#define ENABLE_UART_DEBUG   0

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

#define IR_PIN	GP4
#define FIRE GP0                            // pin for 2n2222

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

uint8_t i;
unsigned long ir_code;
unsigned int address;
unsigned short command;
#if ENABLE_UART_DEBUG
char str3[8];
#endif

//http://www.techdesign.be/projects/011/011_waves.htm

// delay us must be multiplied with 1.25 to get right delay

short samsung_remote_read()
{
    unsigned long count = 0, i;

	// Check ~4.5ms pulse START BIT (remote control sends logic low)
    while ((IR_PIN == 0) && (count < 90)) {
        count++;
        __delay_us(40);
    }

    count = 0;
    // Check 4.5ms SPACE (remote control sends logic low)
    while (IR_PIN && (count < 90)) {
        count++;
        __delay_us(40);
    }

    // Read code message (32-bit)
    for (i = 0; i < 32; i++)
    {
        count = 0;
        while ((IR_PIN == 0) && (count < 15)) {
            count++;
            __delay_us(30);
        }

        if ( (count > 14) || (count < 2))     // samsung protocol?
            return 0;

        count = 0;
        while (IR_PIN && (count < 30)) {
            count++;
            __delay_us(30);
        }

        if ( (count > 29) || (count < 4))     // samsung protocol?
            return 0;

        if (count > 20)                       // If space width > 1ms
            ir_code |= 1ul << (31 - i);       // Write 1 to bit (31 - i)
        else                                  // If space width < 1ms
            ir_code &= ~(1ul << (31 - i));    // Write 0 to bit (31 - i)
    }

    return 1;
}

void main(void)
{
	OSCCAL = __osccal_val();    // Load Oscillator Calibration
	ANSEL  = 0x00;              // Digital IOs only
	TRISIO = 0x10;              // GP4 input, rest all output
	CMCON  = 0x07;		    // Shut off the Comparator, so that pins are available for ADC
	VRCON  = 0x00;	            // Shut off the Voltage Reference for Comparator
	OPTION_REG = 0x80;          // Disable pull-ups
    
	FIRE = 0;

#if ENABLE_UART_DEBUG
	// Intialize Soft UART
	UART_TX = 1;                // TX pin is high in idle state	GP1=1
	UART_TX_DIR = 0;            // set TRISIO1=0 as output direction
#endif

	while(1)
	{
		if (samsung_remote_read())
		{
			address = ir_code >> 16;
			command = ir_code >> 8;
            
			switch(command)
			{
				case 57490:
					FIRE = 1;
					__delay_ms(200);
					FIRE = 0;
					break;
                
				default:
					break;
			}

#if ENABLE_UART_DEBUG
			UART_Transmit('C');
			UART_Transmit('O');
			UART_Transmit('D');
			UART_Transmit(':');
			UART_Transmit(' ');

			for (i=0; i<8; i++) str3[i] = '\0';
			sprintf(str3, "%u", address);

			for (i=0; i<8; i++)
			{
				if (str3[i] == '\0') break;
				UART_Transmit(str3[i]);
			}

			UART_Transmit(' ');
        
			for (i=0; i<8; i++) str3[i] = '\0';
			sprintf(str3, "%u", command);
        
			for (i=0; i<8; i++)
			{
				if (str3[i] == '\0') break;
				UART_Transmit(str3[i]);
			}
            
			UART_Transmit(0x0D);
			UART_Transmit(0x0A);
#endif
		}
	}
}
