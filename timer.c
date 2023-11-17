#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "servo.h"
#include "temp.h"
#include "lcd.h"

extern volatile int current_temp_16;
volatile char timer1_running;

void timer1_init() 
{
    // set timer to CTC mode 
    TCCR1B |= (1 << WGM12); 

    // enable interrupt
    TIMSK1 |= (1 << OCIE1A);

    // 4 seconds at prescalar of 1024
    OCR1A = 62500;
}

/**
 * Starts timer or restarts if currently running
*/
void timer1_start()
{
    // reset count
    TCNT1 = 0;

    timer1_running = 1;

    // set prescalar 1024 to start timer
    TCCR1B |= (0b101 << CS10);
}

ISR(TIMER1_COMPA_vect)
{
    // sets servo back to current temp
    update_servo(get_rounded_temp_F(current_temp_16));

    // turns off timer
    TCCR1B &= ~(0b101 << CS10);

    timer1_running = 0;
}