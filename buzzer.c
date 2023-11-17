#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define TIMER_MAX_COUNT 523 // half second tone
volatile int timer_count = 0;

void timer0_init(void)
{
    // set timer to CTC
    TCCR0A |= (1 << WGM01);

    // enable interrupt
    TIMSK0 |= (1 << OCIE0A);

    // modulo value: frequency (period) of 523 w/ prescalar of 64
    OCR0A = 239;
}

void play_note(void)
{
    timer_count = 0;

    // set prescalar to start timer
    TCCR0B |= (1 << CS01) | (1 << CS00);
}

ISR(TIMER0_COMPA_vect)
{
    PORTB ^= (1 << PB5);

    if (++timer_count == TIMER_MAX_COUNT) {
        // clear prescalar to stop timer
        TCCR0B &= ~((1 << CS01) | (1 << CS00));
    }
}