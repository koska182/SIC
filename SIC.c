/*
 * SIC.c
 *
 * Created: 2.6.2015. 17:29:51
 *  Author: Aleksandar Kosanovic
 */ 

#include <avr/io.h>
/*http://homepage.hispeed.ch/peterfleury/lcdlibrary.zip*/
#define F_CPU 8000000 
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
/*
 16x2 LCD display.  The LiquidCrystal library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver.
 
 read a rotary encoder with interrupts
 Encoder hooked up with common to GROUND,
 encoder0PinA to pin PD2, encoder0PinB to pin PD4
 it doesn't matter which encoder pin you use for A or B  

  The circuit:
 * LCD RS pin to PB4
 * LCD Enable pin to PB5
 * LCD R/W pin to PD6
 * LCD D4 pin to PD7
 * LCD D5 pin to PB0
 * LCD D6 pin to PB1
 * LCD D7 pin to PB2
 * 10K trimmer:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
ADC input on PC0 
Button input on PC1
PWM output on PB3 (OCR2, compare output from Timer2)
 
 */

#define Kp 8		//factor in p part of controller
#define Ki 0.1		//factor in i part of controller
#define Kd 6		//factor in d part of controller
volatile uint16_t encPos = 320;
volatile uint16_t update_lcd = 1;
volatile uint8_t button_down;
static inline void debounce(void);

volatile uint16_t iState =0;
volatile uint16_t dState =0;

uint16_t UpdatePID(int16_t error, uint16_t position)
{
	int16_t pTerm, dTerm, iTerm; 
	int16_t result;

	pTerm = Kp * error;   // calculate the proportional term

	// calculate the integral state with appropriate limiting
	iState += error;
	if (iState > 255) iState = 255;
	else if (iState < 0) iState = 0;

	iTerm = Ki * iState;  // calculate the integral term

	dTerm = Kd * (dState - position);
	dState = position;
	
	result =pTerm + dTerm + iTerm;
	if (result > 255){
		result = 255;
	}
	else if (result<0){
		result = 0;
	}

	return result;
}

int main(void)
{
  float temperature;
  uint8_t heater_state=0; 
  DDRC &= ~(1<<PC1);   //PC1 set to input (button)
  // Routine to setup ADC
  uint16_t adc_value;
  DDRC &= ~(1<<PC0);	//PC0 set to input
  ADMUX = (1<<REFS0);	// ADC input channel set to PC0, Vref =VAcc
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
  // ADEN: Set to turn on ADC , by default it is turned off 
  //ADPS2: ADPS2 and ADPS0 set to make division factor 32 
  
  
  // Routine to setup INT0
  // Assure that pin PD2 (INT0) and PD4 are inputs
  DDRD &= ~(1<<PD4);
  DDRD &= ~(1<<PD2);
  MCUCR |= (1 << ISC01);    // set INT0 to trigger on ANY logic change
  GICR |= (1 << INT0);      // Turns on INT0
  sei();                    // turn on interrupts

  // Routine to setup PWM
  DDRB |= (1 << DDB3);        // PB3 is now an output
  OCR2 = 0;                 // set PWM for 0% duty cycle
  TCCR2 |= (1 << COM21);      // set none-inverting mode
  TCCR2 |= (1 << WGM21) | (1 << WGM20);   // set fast PWM Mode
  TCCR2 |= (1 << CS22)|(0 << CS21)|(1 << CS20);       // set prescaler to 128 and starts PWM 8/(128*256)=244 Hz
   
  lcd_init(LCD_DISP_ON); /* initialize lcd, display on, cursor on
                                  for more options for
                                  lcd_init(), view lcd.h file*/
  
  /*create custom char*/
  uint8_t stupanj[8]={
		 0b00110,
		 0b01001,
		 0b01001,
		 0b00110,
		 0b00000,
		 0b00000,
		 0b00000,
		 0b00000
  };
  	lcd_command(0x40);
  	for (uint8_t i= 0; i<8;i++){
	  	lcd_data(stupanj[i]);
  	}
  	_delay_ms(10);
  
  
  char tempbuff[10];
  lcd_clrscr();             /* clear screen of lcd */
  lcd_home();               /* bring cursor to 0,0 */
  lcd_puts("Hello");        
  lcd_gotoxy(0,1);          /* go to 2nd row 1st col */
  lcd_puts("SIC");  
  _delay_ms(1000);            /* wait 1 s */
  lcd_home();
  lcd_puts("Set Temp: "); //10 znakova
  //lcd_puts(" ");
  itoa(encPos, tempbuff,10);
  lcd_puts(tempbuff);
  lcd_puts(" ");
  lcd_data(0); //°
  lcd_puts("C");
  lcd_gotoxy(0,1);
  lcd_puts("Temp: "); //6 znakova
  lcd_gotoxy(9,1);
  lcd_puts(" ");
  lcd_data(0); //°
  lcd_puts("C");

  while(1)                      /* run continuously */
    {
		debounce();
		if (button_down)
		{ 
			button_down = 0;
			heater_state ^= (1 << 0);
			iState = 0;
		}
		if (update_lcd){
			lcd_gotoxy(10,0);
			itoa(encPos, tempbuff,10);
			lcd_puts(tempbuff);
			update_lcd=0;
		}
      ADCSRA |= (1<<ADSC); // Start conversion
      while (ADCSRA & (1<<ADSC)); // wait for conversion to complete
      adc_value = ADCW;
	  temperature = adc_value*0.1744;
	  temperature = (temperature+2.4548)*4.2688;
	  uint16_t temp = (uint16_t) temperature;
	    
	  uint16_t res = UpdatePID(encPos - temp,temp);
	  
	  if (heater_state){
		OCR2 = res;
		lcd_gotoxy(14,1);
		lcd_puts("ON");
	  }
	  else{
		OCR2 = 0;
		lcd_gotoxy(14,1);
		lcd_puts("  ");	  
	  }
	lcd_gotoxy(6,1);
	itoa(temp, tempbuff, 10);
	lcd_puts(tempbuff);
	_delay_ms(30);
	}
    
    
}
ISR (INT0_vect)
{
  // When an interrupt occurs, we only have to check the level of
	// of pin PD4 to determine the direction
	if (PIND & _BV(PD4)){
	// Increase enc
		if (encPos<450){
			encPos++;
			update_lcd=1;
			}
	}
	else{
		if (encPos>200){
		// Decrease enc
		encPos--;
		update_lcd=1;
		}
	}
}

static inline void debounce(void)
{
	// Counter for number of equal states
	static uint8_t count = 0;
	// Keeps track of current (debounced) state
	static uint8_t button_state = 0;
	// Check if button is high or low for the moment
	uint8_t current_state = (~PINC & (1<<PC1)) != 0;
	if (current_state != button_state) {
		// Button state is about to be changed, increase counter
		count++;
		if (count >= 4) {
			// The button have not bounced for four checks, change state
			button_state = current_state;
			// If the button was pressed (not released), tell main so
			if (current_state != 0) {
				button_down = 1;
			}
			count = 0;
		}
		} else {
		// Reset counter
		count = 0;
	}
}
