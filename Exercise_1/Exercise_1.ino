#include <avr/io.h>                   // pin/port/timer registers
#include <avr/interrupt.h>            // ISR(), sei()
#include <avr/sleep.h>                // sleep_mode utilities
#include <avr/power.h>                // power (optional savings)
#include <util/delay.h>               

int main(void) {
  DDRB  |= (1 << PB5);            // make PB5 an output
  PORTB &= ~(1 << PB5);           // Initially LED Off
  power_adc_disable();
  TCCR1A = 0;                         // normal pin operation
  TCCR1B = (1 << WGM12)               // CTC mode (OCR1A is TOP)
         | (1 << CS12) | (1 << CS10); // prescaler /1024
  OCR1A  = 15624;                     // counts 0..15624 = 15625 ticks => ~1 s
  TIMSK1 = (1 << OCIE1A);             // enable Compare Match A interrupt
  set_sleep_mode(SLEEP_MODE_IDLE);    // choose sleep type (see note)
  sei();                              // global interrupt enable

  while (1) {
    sleep_mode();                     // CPU sleeps here; Timer1 wakes it

    PORTB |=  (1 << PB5);         // LED ON
    _delay_ms(100);                   
    PORTB &= ~(1 << PB5);         // LED OFF
  }
}

ISR(TIMER1_COMPA_vect) {
}