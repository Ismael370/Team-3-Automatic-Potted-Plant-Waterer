#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
//#include <avr/interrupt.h>

int sleep_count = 0;
bool minute_flag = false;

ISR(WDT_vect)
{
  sleep_count++;
  
  if(sleep_count == 7)
  {
    minute_flag = true;
    sleep_count = 0;
  }
  else
    minute_flag = false;
    
  wdt_disable();
}

void wdt_init()
{
  WDTCSR = (1<<WDCE) | (1<<WDE); //Allow changes to timer
  WDTCSR = (1<<WDIE) | (1<<WDP0) | (1<<WDP3); //Set timer for 8 secs
  WDTCSR |= (1<<WDIE); //Enable watchdog interrupts
  wdt_reset(); //Start timer
}

void test()
{
  Serial.begin(9600);
  Serial.println(sleep_count);
  if(minute_flag)
    Serial.println("One minute has passed");
    
  Serial.flush();
}

int main()
{ 
  while(1)
  {
    test();
    
    ADCSRA = 0; //disable adc 
    MCUSR = 0;
    wdt_init(); //initialize watchdog timer
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    noInterrupts();
    sleep_enable();

    //disable brown-out detection
    MCUCR = bit (BODS) | bit (BODSE);
    MCUCR = bit (BODS); 
    
    interrupts();
    sleep_cpu();//put Arduino to sleep
    
    sleep_disable();
  }
}
