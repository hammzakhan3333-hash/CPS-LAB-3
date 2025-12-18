#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define LED_BIT PB5                // LED on PB5 (D13)

volatile uint8_t led_pulse_request = 0;   // request flag from Timer1

void timer1_init_1Hz(void) {
    TCCR1A = 0;                         // normal pin operation
    TCCR1B = (1 << WGM12)               // CTC mode (OCR1A is TOP)
           | (1 << CS12) | (1 << CS10); // prescaler /1024
    OCR1A  = 15624;                     // counts 0..15624 = ~1s
    TIMSK1 = (1 << OCIE1A);             // enable Compare Match A interrupt
}
void timer0_start_100ms(void) {
    TCCR0A = (1 << WGM01);              // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00); // prescaler /64
    OCR0A  = 249;                       // 1 ms tick
    TIMSK0 |= (1 << OCIE0A);            // enable Compare Match A interrupt
}

void timer0_stop(void) {
    TCCR0B = 0;                         // stop Timer0
    TIMSK0 &= ~(1 << OCIE0A);           // disable Timer0 interrupt
}
ISR(TIMER1_COMPA_vect) {
    led_pulse_request = 1;              // signal main loop to activate LED
}
ISR(TIMER0_COMPA_vect) {
    static uint8_t ms_count = 0;
    ms_count++;

    if (ms_count >= 100) {              // reached 100 ms
        PORTB &= ~(1 << LED_BIT);       // LED OFF
        ms_count = 0;
        timer0_stop();                  // stop Timer0
    }
}
int main(void) {
    DDRB  |= (1 << LED_BIT);            // make PB5 an output
    PORTB &= ~(1 << LED_BIT);           // Initially LED Off

    power_adc_disable();                // disable ADC (not used)

    timer1_init_1Hz();                  // initialize Timer1 (1s)
    set_sleep_mode(SLEEP_MODE_IDLE);    // choose IDLE sleep mode
    sei();                              // global interrupt enable

    while (1) {
        sleep_mode();                   // CPU sleeps here; ISRs wake it

        if (led_pulse_request) {        // flag from Timer1 ISR
            led_pulse_request = 0;

            PORTB |= (1 << LED_BIT);    // LED ON
            timer0_start_100ms();        // start 100 ms timer
        }
    }
}
