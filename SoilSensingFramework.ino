#define F_CPU 16000000UL
#define FS 10000
#include <avr/io.h>
#include <avr/interrupt.h>

float ADC_soil;

ISR(TIMER2_COMPA_vect)
{
  ADCSRA |= (1<<ADSC);
}

ISR(ADC_vect)
{
  ADC_soil = ADC;
}

int main()
{
  Serial.begin(9600);
  ADC_init(); 
  IO_init();
  uint8_t state = 1;
  
  while(1)
  {
    Serial.print(ADC_soil);
    //If Low demand is selected and the threshold is met
    if(ADC_soil < 300 && state == 1)
      Serial.println("Pump ON");
    //If Medium demand is selected and the threshold is met 
    else if(ADC_soil < 700 && state == 2)
      Serial.println("Pump ON");
    //If Medium demand is selected and the threshold is met
    else if(ADC_soil < 900 && state == 3)
      Serial.println("Pump ON");
    //If the threshold is not met for the current demand
    else
      Serial.println("Pump OFF");
  }
}

void ADC_init()
{
  //Timer2
  TCCR2A |= (1 << WGM21); //CTC mode
  OCR2A = 50; //100us or 10kHz       256/512 = x/100
  TCCR2B |= (1 << CS20) | (1 << CS21); //pre-scale 32 or 512us at 256
  
  //ADC
  ADMUX |= (1 << REFS0); //VCC as VREF
  //ADMUX |= (1 << MUX0) | (1 << MUX2); //ADC0 or A0/PINC0
  ADCSRA |= (1 << ADEN); //ADC enable 
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1); //pre-scale 8 conversion takes 500ns

  //Interrupts
  TIMSK2 |= (1 << OCIE2A); //Timer2 compare A
  ADCSRA |= (1 << ADIE); //ADC conversion complete
  sei(); //Enable global interrupts
}

void IO_init()
{
  DDRD |= (1<<DDD2) | (1<<DDD3) | (1<<DDD4); //Set pins D2 - D4 as outputs for LEDS
}

// Lowpass FIR/IIR Filter using floating point. This is a floating point 
// implementation of the the filter described on Pages 171 and 172
// of "Ten Essential Skills for Electrical Engineers" Dorr, 2014" ISBN 978-1-118-52742-9
#define FCutoff 400   // 3 dB frequency in Hz
#define ALPHA   ((float)(1 - 2.0*PI*FCutoff/FS))

float FirIirLpfFloat (float input) {
  static float FilterReg = 0.0;
  float LastReg;
  
  LastReg = FilterReg;
  FilterReg = input + ALPHA * FilterReg;
  return (FilterReg + LastReg)*(1.0 - ALPHA)/2.0;
}
