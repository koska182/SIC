
/*

 16x2 LCD display.  The LiquidCrystal library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver.
 
 read a rotary encoder with interrupts
 Encoder hooked up with common to GROUND,
 encoder0PinA to pin 2, encoder0PinB to pin 4
 it doesn't matter which encoder pin you use for A or B  

  uses Arduino pullups on A & B channel outputs
  turning on the pullups saves having to hook up resistors 
  to the A & B channel outputs 
 
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4 // zamijeniti
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2 // zamijeniti
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 
 */
// include the library code:
#include <LiquidCrystal.h>
#include <PID_v1.h>
#define encoder0PinA  2
#define encoder0PinB  4
#define ptcPin 3 //Ptc input from heater
#define heater 2 //PWM output to heater
#define button 5 //button for standby
boolean heaterState = false;
volatile unsigned int encoder0Pos = 320;
volatile unsigned int error;
int temperature;
int pwmrate;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  pinMode(ptcPin, INPUT);
  pinMode(heater, OUTPUT);  
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
  delay (1000);
  temperature = analogRead(ptcPin); //dorada
  lcd.home();
  lcd.print("Set Temp: "); //10 znakova
  lcd.print(encoder0Pos); // 3 znaka
  lcd.print(" °C"); // 3 znaka
  lcd.setCursor(0, 1);
  lcd.print("Temp: "); //6 znakova
  lcd.print(temperature); // 3 znaka
  lcd.print(" °C"); // 3 znaka
  myPID.SetMode(AUTOMATIC); 
}

void loop() {
  if (digitalRead(button) == LOW){
    delay (50);
    if (digitalRead(button) == LOW){
      heaterState = !heaterState;}
  }
  
  // set the cursor to column 6, line 0
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(6, 0);
  // print the encoder position:
  lcd.print(encoder0Pos);
  temperature = analogRead(ptcPin); // dorada
  lcd.setCursor(4, 1);
  lcd.print(temperature);
  
  Setpoint = encoder0Pos;
  Input = temperature;
  myPID.Compute();
  pwmrate = Output;
  if (heaterState){
    digitalWrite(heater, LOW);}
   else {
    analogWrite(heater, pwmrate);
   }
}






/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B  

   uses Arduino pullups on A & B channel outputs
   turning on the pullups saves having to hook up resistors 
   to the A & B channel outputs 
*/ 

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    
    if (encoder0Pos < 450){
    encoder0Pos++;}
  } else {
    if (encoder0Pos > 200){
    encoder0Pos--;}
  }
}



