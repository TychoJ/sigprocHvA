#define F_CPU 32000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "clock.h"

// Dit programma leest d.m.v de ADC de spanning op PIN A2.
// Dit is de input van je digitale filter. Dit filter moet je zelf bouwen.
// De output van je digitale filter wordt d.m.v. de DAC op PIN A10 gezet.
// De samplefrequentie van deze routine wordt bepaald door een timer interrupt.


//ADC_voltage:
#define MAX_VALUE_ADC   2047                               // only 11 bits are used
#define VCC			(float) 3.3
#define VREF		(float) VCC / 1.6

#define N		8UL
#define F_CLK	1000UL
#define TC_PER	(F_CPU / ((N * F_CLK)) - 1)

//DAC
#define MAX_VALUE_DAC 4095
#define CAL_DAC 1.000  // Calibratiewaarde DAC

#define ADC2DAC (float)(VREF * MAX_VALUE_DAC * CAL_DAC) / ((MAX_VALUE_ADC + 1) * VCC)

#define FIR_LAYERS 19

#define Fc		70
#define F0s1	1.0688
#define F0s2	0.6265
#define as1		0.5861

#define T		1e-3
#define T2		T * T

#define wcs1	2 * Fc * M_PI / F0s1
#define wcs2	2 * Fc * M_PI / F0s2

#define a10		4 / T2 + as1 * wcs1 * 2 / T + wcs1*wcs1
#define a11		2 * wcs1 * wcs1 - 8 / T2
#define a12		4 / T2 - as1 * wcs1 * 2 / T + wcs1*wcs1
#define a20		2 / T + wcs2
#define a21		wcs2 - 2 / T

#define b10		4 / T2
#define b11		-8 / T2
#define b12		4 / T2
#define b20		-2 / T
#define b21		2 / T

float b1[] = {b10, b11, b12};
float b2[] = {b20, b21};

float a1[] = {a10, a11, a12};
float a2[] = {a20, a21};


void init_timer(void){
	TCE0.CTRLB     = TC_WGMODE_NORMAL_gc;	// Normal mode
	TCE0.CTRLA     = TC_CLKSEL_DIV8_gc;	// prescaling 8 (N = 64)
	//TCE0.INTCTRLA  = TC_OVFINTLVL_MED_gc;	// enable overflow interrupt medium level
	TCE0.PER       = TC_PER;	//(62499/2)*0.002;			// sampletijd ( (62499/2) = 1 sample/seconde).
}

void init_adc(void){
	PORTA.DIRCLR		= PIN2_bm|PIN3_bm;
	ADCA.CH0.MUXCTRL	= ADC_CH_MUXPOS_PIN2_gc |	// PA2 (PIN A2) to + channel 0
	ADC_CH_MUXNEG_GND_MODE3_gc;						// GND to - channel 0
	ADCA.CH0.CTRL	= ADC_CH_INPUTMODE_DIFF_gc;		// channel 0 differential mode
	ADCA.REFCTRL	= ADC_REFSEL_INTVCC_gc;
	ADCA.CTRLB		= ADC_RESOLUTION_12BIT_gc |
	ADC_CONMODE_bm;				// signed conversion
	ADCA.PRESCALER	= ADC_PRESCALER_DIV16_gc;
	ADCA.EVCTRL		= ADC_SWEEP_0_gc |			// sweep channel 0
	ADC_EVSEL_0123_gc |		// select event channel 0, 1, 2, 3
	ADC_EVACT_CH0_gc;			// event system triggers channel 0
	EVSYS.CH0MUX	= EVSYS_CHMUX_TCE0_OVF_gc;	// Timer overflow E0 event
	ADCA.CH0.INTCTRL	= ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc; // enable ADCA CH0 interrupt when conversion is complete
	ADCA.CTRLA		= ADC_ENABLE_bm;
}

void init_dac(void){
	DACB.CTRLC = DAC_REFSEL_AVCC_gc;
	DACB.CTRLB = DAC_CHSEL_SINGLE_gc;
	DACB.CTRLA = DAC_CH0EN_bm | DAC_ENABLE_bm;
}


//ISR(TCE0_OVF_vect){				//TIMER Interupt
// 	int16_t  voltage ;		// contains read in voltage (mVolts)
//PORTC.OUTTGL = PIN0_bm;	//Toggle the LED

//ADCA.CH0.CTRL |= ADC_CH_START_bm; // start conversion
// 	voltage = (double) res * 1000 * VREF / (MAX_VALUE_ADC + 1);					// Measured voltage in Volts.
// 	if (voltage < 0) voltage = 0;
//}

ISR(ADCA_CH0_vect){
	PORTC.OUTTGL = PIN0_bm;	//Toggle the LED
	uint32_t BinaryValue; 	// contains value to write to DAC
	
	
	//	<Jochem code>
	
	static float x0[3] = {0};
	static float x1[3] = {0};
	static float x2[2] = {0};
	static float y[2] = {0};
		
	static uint8_t xIndex = 0;
	static uint8_t yIndex = 0; //Wordt ook gebruikt voor x2.
	
	x0[xIndex] = (float)ADCA.CH0.RES;
	x1[xIndex] = 0;
	
	// Eerste deel
	for(uint8_t i = 1; i < 3; i++) {
		x1[xIndex] += b1[2 - i] * x0[(xIndex - i) % 3];
		// Het meest recente monster moet vermenigvuldigt worden met b12, daarna b11, dan b10
	}
	
	for (uint8_t i = 1; i < 3; i++) {
		x1[xIndex] += -a1[i] * x1[(xIndex - i) % 3];
	}
	x1[xIndex] /= a10;
	
	// Tweede deel
	// Dit kan modulairder maar dan wordt het minder snel.
	// Modulairder is een mooi woord.
	
	x2[yIndex] = b21 * x1[xIndex] + b20 * x2[!xIndex];
	y[yIndex]  = (x2[yIndex] + -a21 * y[!yIndex]) / a20;

	int16_t out = (int16_t) y[yIndex];

	xIndex = (xIndex + 1) % 3;
	yIndex = !yIndex; 
	// Hij moet gewoon wisselen tussen 1 en 0 dus dan is dit sneller
	
	//	</Jochem code>
	
	BinaryValue = out * ADC2DAC; //Bitwaarde
// 	printOut = ADC2DAC;
	DACB.CH0DATA = BinaryValue;			//write &USBDataIn to DAC (PIN A10)
	while (!DACB.STATUS & DAC_CH0DRE_bm);
}


int main(void){
	PORTC.DIRSET = PIN0_bm;	// the LED (bit 0 on port C) is set as output.
	PORTB.DIRSET = PIN2_bm;
	PORTF.DIRSET = PIN0_bm | PIN1_bm;

	init_clock();
	init_timer();			// init timer
	init_dac();				// init DAC
	init_adc();				// init ADC
		
	PMIC.CTRL     |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;		// set low and medium level interrupts
	sei();					//Enable interrupts
	
	while (1) {
		asm volatile("nop");
	}
}