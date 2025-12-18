#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define LED_BIT PB5                // LED on PB5 (D13)

volatile uint8_t wake_flag = 0;    // flag set by INT0 ISR

void adc_init(void) {
    ADMUX  = (1 << REFS0);                     // AVcc reference, ADC0 input
    ADCSRA = (1 << ADEN)                       // enable ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler /128
}

uint16_t adc_read_A0(void) {
    ADCSRA |= (1 << ADSC);                     // start conversion
    while (ADCSRA & (1 << ADSC));              // wait until done
    return ADC;                                // return result
}
void int0_init(void) {
    DDRD  &= ~(1 << PD2);                      // PD2 as input
    PORTD |=  (1 << PD2);                      // enable pull-up

    EICRA &= ~((1 << ISC01) | (1 << ISC00));   // low-level trigger (for power-down)
    EIMSK |=  (1 << INT0);                     // enable INT0
}
ISR(INT0_vect) {
    wake_flag = 1;                             // signal main loop
}
int main(void) {
    DDRB  |= (1 << LED_BIT);                   // make PB5 an output
    PORTB &= ~(1 << LED_BIT);                  // Initially LED Off

    adc_init();                                // initialize ADC
    int0_init();                               // initialize INT0

    // disable unused peripherals for lower power
    power_spi_disable();
    power_twi_disable();
    power_timer0_disable();
    power_timer1_disable();
    power_timer2_disable();

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);       // deepest sleep mode
    sei();                                     // global interrupt enable

    while (1) {
        sleep_enable();
        sleep_cpu();                           // MCU sleeps here
        sleep_disable();

        if (wake_flag) {                       // woke due to button
            wake_flag = 0;

            PORTB |= (1 << LED_BIT);           // LED ON
            uint16_t value = adc_read_A0();    // read A0
            (void)value;                       

            for (volatile uint32_t i = 0; i < 100000; i++);

            PORTB &= ~(1 << LED_BIT);          // LED OFF
        }
    }
}
