#include <avr/sleep.h>
#include <avr/interrupt.h>

ISR(PCINT1_vect)
{
  Serial.println("Awake");
  Serial.flush();
  
  if(!(PINC & (1<<PINC1)))
  {
    Serial.println("A1");
  }
  else if(!(PINC & (1<<PINC2)))
  {
    Serial.println("A2");
  }
  else if(!(PINC & (1<<PINC3)))
  {
    Serial.println("A3");
  }
  
  Serial.flush();
}

void btn_init()
{
  DDRC &= ~(1<<DDC1) & ~(1<<DDC2) & ~(1<<DDC3); //Set pins A1, A2, and A3 as inputs
  PORTC |= (1<<PORTC1) | (1<<PORTC2) | (1<<PORTC3); //Enable pull-up resistors

  PCICR |= (1<<PCIE1); //Set Pin Change Interrupts for Port C (A0-A5)
  PCMSK1 |= (1<<PCINT9) | (1<<PCINT10) | (1<<PCINT11);//Enable only pins A1, A2, and A3 for interrupt
  sei();
}

int main()
{
  btn_init();
  Serial.begin(9600);
  while(1)
  {
    ADCSRA = 0; //disable adc 
    MCUSR = 0;

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();

    //disable brown-out detection
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS); 
    
    sei();
    Serial.println("Going to sleep");
    Serial.flush();
    sleep_cpu();//put Arduino to sleep

    sleep_disable();
  }
  return 0;
}
