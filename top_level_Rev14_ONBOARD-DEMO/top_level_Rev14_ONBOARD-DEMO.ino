/*
 * PLANT'S AUTOMATED WATERING (PAW) SYSTEM
 * SDSU - Senior Design
 * Authors: Team 3 - Ismael Chavez, Dhiaa Bantan, Daniel Gish, Matt Lipscomb, and Ebrahim Almershed
 * Revision 11 - 10.29.2021
 * 
 */


#define F_CPU 16000000UL
#define MEM (uint8_t*)10
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <LiquidCrystal.h>

// LCD Pin
LiquidCrystal lcd(5, 6, 13, 12, 11, 10);

// Motor Pin Associations
int motor_enable = 3;
int motor_forward = 4;
int driver_pwr = 8;

// Reservoir Sensor Pin Associations
int r_sensor_pin = 9;

// Intermediate (Software Only) Variables
int sleep_count = 0;        // Counts the iterations of 8 second sleep cycles performed by the system in SLEEP_MODE_PWR_DOWN
bool r_full = true;         // HIGH means it is full. LOW means it is empty.
bool sleep = true;          // Global flag to indicate when the device is asleep
bool THR_flag = 0;          // Global flag to indicate if the threshold is set to the low one (0) or the high one (1)
bool pump_on = false;       // Global flag to indicate the motor's status
bool demo_mode = false;
bool demo_mode_toggle = false;

// Threshold Constants & Variable
unsigned int LOW_THR_lo = 560; 
unsigned int LOW_THR_hi = 460;
unsigned int NORM_THR_lo = 530;
unsigned int NORM_THR_hi = 410;
unsigned int HI_THR_lo = 520;
unsigned int HI_THR_hi = 370;
unsigned int THR;
unsigned int sleep_duration = 4;

// Mode Selection Initialization
uint8_t curr_state = 1;

// Calculations Variables for Soil Moisture Readings
float ADC_soil;


// USER MODE SELECTION ISR
// Default Threshold level is high for each mode
ISR(PCINT1_vect) {
  if( !(PINC & (1<<PINC3)) && !(PINC & (1<<PINC4)) && !(PINC & (1<<PINC5)) ) {
    demo_mode_toggle = true;             // DEMO mode
  }
  else if(!(PINC & (1<<PINC3))) {
    eeprom_update_byte(MEM, (uint8_t)1); // LOW mode
  }
  else if(!(PINC & (1<<PINC4))) {
    eeprom_update_byte(MEM, (uint8_t)2); // NORMAL mode
  }
  else if(!(PINC & (1<<PINC5))) {
    eeprom_update_byte(MEM, (uint8_t)3); // HIGH mode
  }
  setTHR_High();
}


// TEN-MINUTE SLEEP TIMER ISR
ISR(WDT_vect) {
  sleep_count++;
  
  if(sleep_count == sleep_duration) {     // If it's been 10 minutes, the device wakes up (FOR TESTS, USE 4 --> 32 seconds. For final revision, 75 --> 10 minutes)
    sleep = false;
    sleep_count = 0;
  }
  else if( sleep_count >= 2 && pump_on == true ) {    // If it's been 16 seconds and the pump is on, turn the pump off and reset the 10 minute timer
    //sleep = false;
    pump_OFF();
    sleep_count = 0;
    sleep = true;
  }
  else {    // If either previous conditions are true, keep sleeping
    sleep = true;
  }

}


// INITAILIZE ADC FUNCTION
void ADC_init() {
  ADMUX |= (1 << REFS0);                                                // VCC as VREF
  //ADMUX &= ~(1 << MUX3) | ~(1 << MUX2) | ~(1 << MUX1) | ~(1 << MUX0); // Option of explicitly setting ADC input from ADC0 or A0/PINC0
  ADCSRA |= (1 << ADEN);                                                // ADC enable 
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);                 // Pre-scale 128 for 125kHz sampling rate, allowing 10-bits of resolution -- takes about 8us for conversion

}


// INITIALIZE BUTTONS AND PIN ASSIGNMENTS
void init_btns() {
  DDRC &= ~(1<<DDC3) & ~(1<<DDC4) & ~(1<<DDC5);         // Set pins A3, A4, and A5 as inputs
  PORTC |= (1<<PORTC3) | (1<<PORTC4) | (1<<PORTC5);     // Enable pull-up resistors

  PCICR |= (1<<PCIE1);                                  // Set Pin Change Interrupts for Port C (A0-A5)
  PCMSK1 |= (1<<PCINT11) | (1<<PCINT12) | (1<<PCINT13); // Enable only pins A3, A4, and A5 for interrupt
}


// INITIALIZE WATCHDOG TIMER FUNCTION
void wdt_init() {
  WDTCSR = (1<<WDCE) | (1<<WDE);              // Allow changes to timer
  WDTCSR = (1<<WDIE) | (1<<WDP0) | (1<<WDP3); // Set timer for 8 secs
  WDTCSR |= (1<<WDIE);                        // Enable watchdog interrupts
  wdt_reset();                                // Start timer
}


// INITIALIZE SLEEP MODE FUNCTION
void sleep_init() {
    MCUSR = 0;
    //ADCSRA = 0; // Disable ADC when asleep -- keep commented without ISRs
    wdt_init(); // Initialize watchdog timer
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    noInterrupts();
    sleep_enable();

    // Disable brown-out detection
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS); 
    
    interrupts();
    sleep_cpu();//put Arduino to sleep    
}


// SETUP: INITIALIZE ALL NECESSARY PINS, VARIABLES, AND FUNCTIONS
void setup() {
  
  // Sets required pins to outputs
  pinMode(motor_enable, OUTPUT);    // Controls the water pump motor speed using PWM = 80%. Constant at 200 (out of 256 analog output).
  pinMode(motor_forward, OUTPUT);   // Controls the water pump motor direction. HIGH = ON, LOW = OFF.
  pinMode(driver_pwr, OUTPUT);      // Controls the 5V power to the L293D motor driver. HIGH = OFF, LOW = ON. When OFF, current draw is minimized.
  pinMode(r_sensor_pin, INPUT);     // Digital input pin for Reservoir Sensor

  // LCD Setup
  lcd.begin(16, 2);
  lcd.print("Mode: ");
  lcd.setCursor(0, 1);
  lcd.print("Soil VWC: ");

  // Initialization functions (for interrupts & soil sensor data)
  sei();
  ADC_init();
  init_btns();
  for(int i = 0; i < 3; i++ )
    check_soil();
  
  // Motor's Initial state
  digitalWrite(motor_forward, LOW); // Puts the motor direction as not specified. Ensures the pump is off.
  digitalWrite(driver_pwr, HIGH);   // Puts Motor OFF as initial state
  analogWrite(motor_enable, 255);   // Sets PWM to 80% motor speed

}



//////////////////////////CENTRAL LOOP////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void loop() {

  // Recall Mode Selection
  curr_state = eeprom_read_byte(MEM);   // Check Mode Selection

  // Toggle demo mode if the buttons have been pushed
  if(demo_mode_toggle == true)
    toggle_demo_mode();

  // Update LCD
  LCD_update();

  // Update soil data every wake-up cycle
  check_soil();                         // Check soil moisture & update LCD

  // If the device should not be asleep, run the Control System Code
  if( !sleep )
    waterPumpManager();                   // Apply Feedback Control System


// Otherwise, and after all other operations, go back to sleep
  sleep = true;
  sleep_init();
  
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


// UPDATE LCD DISPLAY TO MODE SELECTION FUNCTION 
void LCD_update() {
    if(curr_state == 1) {
      lcd.setCursor(7, 0);
      lcd.print("Low   ");
    }
    else if(curr_state == 2)
    {
      lcd.setCursor(7, 0);
      lcd.print("Normal");
    }
    else if(curr_state == 3)
    {
      lcd.setCursor(7, 0);
      lcd.print("High  ");
    }
    else
    {
      lcd.setCursor(7, 0);
      lcd.print("N/A   ");
    }   
}


// CHECK RESERVOIR SENSOR FUNCTION
void check_reservoir() {
  r_full = digitalRead(r_sensor_pin);
}


// CHECK SOIL FUNCTION
void check_soil() {
  
  // Start ADC conversion
  ADCSRA |= (1<<ADSC);
  
  // Wait for the conversion to complete (~8us)
  while(ADCSRA & (1<<ADSC)); //as long as ADSC pin is 1 just wait.  /// TEMPORARY??
  ADC_soil = ADC; // Get the latest data from ADC 

  // Auto-correct small over- and under- readings
  if(ADC_soil > 590)
    ADC_soil = 590;
  if(ADC_soil < 175)
    ADC_soil = 175;

  // Convert result to VWC in percent
  int soil_in_VWC = ((590.0-ADC_soil)/415.0)*100.0;   // Converts ADC value to VWC(%)

  // Every time the Soil Moisture is sensed, update the value on the LCD
  lcd.setCursor(10, 1);
  lcd.print(soil_in_VWC); lcd.print("%  ");

}


void toggle_demo_mode() {
  if(demo_mode = false) {
    demo_mode = true;
    sleep_duration = 4;
    LCD_update_demo_mode();
  }
  else if(demo_mode = true) {
    demo_mode = false;
    sleep_duration = 75;
    LCD_update_demo_mode();
  }
  demo_mode_toggle = false;
  _delay_ms(20);
}


// LCD UPDATE FOR DEMO MODE STATUS
void LCD_update_demo_mode() {
    
  // LCD Flash Demo Mode
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DEMO MODE: ");
    if(demo_mode == true)
      lcd.print("ON ");
    else if(demo_mode == false)
      lcd.print("OFF");
    else
      lcd.print("BROKE");
  delay(3000);
  
  // LCD Reset
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mode: ");
  lcd.setCursor(0, 1);
  lcd.print("Soil VWC: ");
}

// SET THRESHOLD TO LOW THRESHOLD FUNCTION
void setTHR_Low() {
  
  THR_flag = 0; // Global threshold flag to indicate whether the threshold is set to the low one or high one. “0” for low threshold and “1” for high threshold.
  
  if(curr_state == 1) {
    THR = LOW_THR_lo;
  }
  else if(curr_state == 2){
    THR = NORM_THR_lo;
  }
  else if(curr_state == 3){
    THR = HI_THR_lo;
  }
}


// SET THRESHOLD TO HIGH THRESHOLD FUNCTION
void setTHR_High() {
  
  THR_flag = 1; // Global threshold flag to indicate whether the threshold is set to the low one or high one. “0” for low threshold and “1” for high threshold.
  
  if(curr_state == 1) {
    THR = LOW_THR_hi;
  }
  else if(curr_state == 2){
    THR = NORM_THR_hi;
  }
  else if(curr_state == 3){
    THR = HI_THR_hi;
  }
}


// MOTOR "ON" FUNCTION
void pump_ON(){
  pump_on = true;
  digitalWrite(motor_forward, HIGH);
  digitalWrite(driver_pwr, LOW); // Turn the water pump driver ON to control the pump
}


// MOTOR "OFF" FUNCTION
void pump_OFF(){
  pump_on = false;
  digitalWrite(driver_pwr, HIGH); // Turn the water pump driver OFF to control the pump
  digitalWrite(motor_forward, LOW);
}


// WATER PUMP FEEDBACK CONTROL SYSTEM W/ HYSTERESIS FUNCTION
void waterPumpManager() 
{
  cli();
  
    // Check the Reservoir Status:
    check_reservoir();
    
    if (r_full == false){   // If the reservoir is empty, keep the pump OFF
      pump_OFF();    
    }
    
    // Notice that the logic will be inverted because our soil sensor sees high ADC readings as dry moisture content.
    else 
    {
      if( THR_flag == 0 && ADC_soil > THR )         // PATH 1: The plant is too dry and the Threshold is LOW (Switch to HIGH Threshold & begin 1st watering cycle)
      {
        // If the reservoir is not empty, check the THR
        // enable the pump for 10s then disable it based on the THR:
        setTHR_High();
        pump_ON();
      }
  
      else if ( THR_flag == 1 && ADC_soil > THR)    // PATH 2: The plant is too dry and the Threshold is HIGH (Water Repeatedly)
      {
        pump_ON();
      }
      else if ( THR_flag == 1 && ADC_soil < THR )   // PATH 3: The plant is too wet & the Threshold is HIGH (Switch to LOW Threshold)
      {
        setTHR_Low();
      }
      else                                          // SHOULD BE SLEEPING: The plant is too wet & the Threshold is LOW (Sleep Repeatedly, wait for soil to "dry out")
      {
        sleep = true; // sleep again for 10 minutes.
      }
    }
  sei();
}
