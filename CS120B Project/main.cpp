/*
 * Final_Project.cpp
 *
 * Created: 11/24/2019 6:51:17 PM
 * Author : Alex Kevakian
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "io.h" 
#include <avr/eeprom.h>

#define customChar 0b00000000

unsigned int EEMEM icon_Robot;

unsigned char customChar_up[8] = { // A up arrow icon
	0b00000,
	0b00000,
	0b00100,
	0b01110,
	0b11111,
	0b00100,
	0b00100,
	0b00100
};

unsigned char customChar_robot[8] = { // A robot icon
	0b00000,
	0b00000,
	0b01010,
	0b01010,
	0b11111,
	0b10101,
	0b11011,
	0b11111
};
// ADC init found on https://www.electronicwings.com/avr-atmega/servo-motor-interfacing-with-atmega16
void ADC_Init()			// ADC Initialization function
{
	DDRA=0x00;		// Make ADC port as input
	ADCSRA = 0x87;		// Enable ADC, with freq/128 
	ADMUX = 0x40;		// Vref: Avcc, ADC channel: 0
}
// ADC read found on https://www.electronicwings.com/avr-atmega/servo-motor-interfacing-with-atmega16
int ADC_Read(char channel)	// ADC Read function 
{
	ADMUX = 0x40 | (channel & 0x07);// set input channel to read
	ADCSRA |= (1<<ADSC);	// Start ADC conversion

	// Wait until end of conversion by polling ADC interrupt flag
	while (!(ADCSRA & (1<<ADIF)));
	ADCSRA |= (1<<ADIF);	// Clear interrupt flag
	_delay_ms(1);		// Wait a little bit
	return ADCW;		// Return ADC word
}

 
int main(void)
{
	ADC_Init();
	
	DDRC = 0xFF; PORTC = 0x00;
	DDRD = 0xFF; PORTD = 0x00;
	DDRB = 0xFF; PORTB = 0x00;
	DDRA = 0x00; PORTA = 0xFF;
	
	unsigned char button1 = 0x00;
	unsigned char button2 = 0x00;
	
	
	
	//-----------------------------------------------------------------------------------------------------------
	// implementation found on https://www.electronicwings.com/avr-atmega/servo-motor-interfacing-with-atmega16
	TCNT1 = 0;		//Set timer1 count zero
	ICR1 = 2499;	// Set TOP count for timer1 in ICR1 register

	// Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clock/64 
	TCCR1A = (1<<WGM11)|(1<<COM1A1);
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
	
	//set OC1A,OC2B on down counting, clear on up counting
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1);
	//------------------------------------------------------------------------------------------------------------
	
	LCD_init();
	
	if (eeprom_read_word(&icon_Robot) == 1) {
		// Display first icon.
		LCD_ClearScreen();
		LCD_Cursor(1);
		LCD_addChar(customChar_robot);
		LCD_WriteData(customChar);
	}
	
	else if (eeprom_read_word(&icon_Robot) == 2) {
		// Display other icon.
		LCD_ClearScreen();
		LCD_Cursor(1);
		LCD_addChar(customChar_up);
		LCD_WriteData(customChar);
	}
	
	
	while(1)
	{
		//set buttons
		button1 = ~PINA & 0x08;
		button2 = ~PINA & 0x10;
		
		//move arm	
		OCR1A = 75 + (ADC_Read(0)/4);
		OCR1B = 75 + (ADC_Read(1)/4);
		
		if(button1){
			eeprom_write_word(&icon_Robot, 1);
			LCD_ClearScreen();
			LCD_Cursor(1);
			LCD_addChar(customChar_robot);
			LCD_WriteData(customChar);
		}
		else if(button2){
			eeprom_write_word(&icon_Robot, 2);
			LCD_ClearScreen();
			LCD_Cursor(1);
			LCD_addChar(customChar_up);
			LCD_WriteData(customChar);
		}
		
		
	
	}
}

