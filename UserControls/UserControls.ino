#define F_CPU 16000000UL
#define MEM (uint8_t*)10
#include <avr/io.h>
#include <avr/eeprom.h>
#include <LiquidCrystal.h>
//LiquidCrystal lcd(6, 7, 8, 9, 10, 11);
LiquidCrystal lcd(5, 6, 13, 12, 11, 10);

void io_init()
{
  DDRD &= ~(1<<DDD2) & ~(1<<DDD3) & ~(1<<DDD4); //Set pins D2 - D4 as inputs for buttons
  PORTD |= (1<<PORTD2) | (1<<PORTD3) | (1<<PORTD4); //Enable pull-up for buttons

  DDRD |=  (1<<DDD6) | (1<<DDD7); //Set pins D6 - D7 as outputs for LCD select signals
  DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3); //Set pins B0 - B3 as outputs for LCD data
}

void read_button()
{   
    if(!(PIND & (1<<PIND2)))
    {
      eeprom_update_byte(MEM, (uint8_t)1); //red
    }
    else if(!(PIND & (1<<PIND3)))
    {
      eeprom_update_byte(MEM, (uint8_t)2); //yellow
    }
    else if(!(PIND & (1<<PIND4)))
    {
      eeprom_update_byte(MEM, (uint8_t)3); //green
    }
}

int main()
{
  io_init();
  lcd.begin(16, 2); //Set # of cols/rows
  lcd.print("Target:");
  lcd.setCursor(0, 1);
  lcd.print("Humidity:");
  Serial.begin(9600);
  uint8_t curr_state;
  
  while(1)
  {
    read_button();
    curr_state = eeprom_read_byte(MEM);
    Serial.println(curr_state);
    
    if(curr_state == 1)
    {
      lcd.setCursor(8, 0);
      lcd.print("Low ");
    }
    else if(curr_state == 2)
    {
      lcd.setCursor(8, 0);
      lcd.print("Med ");
    }
    else if(curr_state == 3)
    {
      lcd.setCursor(8, 0);
      lcd.print("High");
    }
    else
    {
      lcd.setCursor(8, 0);
      lcd.print("N/A ");
    }
  }
}
