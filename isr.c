#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "lcd.h"
#include "ds18b20.h"
#include "servo.h"
#include "timer.h"

extern volatile uint8_t new_state, old_state;
extern volatile char encoder_changed; // Flag for state change
extern volatile char button_changed;

extern volatile char THRESHOLD_SELECT;
extern volatile uint8_t high_thresh;
extern volatile uint8_t low_thresh;

extern volatile char timer1_running;

extern enum {
    NORMAL,
    LOW,
    HIGH,
    BELOW_3,
    ABOVE_3
};

// rotary encoder interrupt routine
ISR(PCINT1_vect)
{
    // Read the input bits and determine A and B.
    uint8_t x = PINC;
    char a = x & (1 << PC1);
    char b = x & (1 << PC2);

    // convert a and b to 1 or 0
    a = (a >> PC1);
    b = (b >> PC2);

    char diff = 0;

    // The following code is for Tasks 4 and later.
    // For each state, examine the two input bits to see if state
    // has changed, and if so set "new_state" to the new state,
    // and adjust the count value.
    if (old_state == 0)
    {
        // Handle A and B inputs for state 0
        if (a == 1)
        {
            new_state = 1;
            diff = -1;
        }
        if (b == 1)
        {
            new_state = 2;
            diff = 1;
        }
    }
    else if (old_state == 1)
    {

        // Handle A and B inputs for state 1
        if (a == 0)
        {
            new_state = 0;
            diff = 1;
        }
        if (b == 1)
        {
            new_state = 3;
            diff = -1;
        }
    }
    else if (old_state == 2)
    {

        // Handle A and B inputs for state 2
        if (a == 1)
        {
            new_state = 3;
            diff = 1;
        }
        if (b == 0)
        {
            new_state = 0;
            diff = -1;
        }
    }
    else
    { // old_state = 3

        // Handle A and B inputs for state 3
        if (a == 0)
        {
            new_state = 2;
            diff = -1;
        }
        if (b == 0)
        {
            new_state = 1;
            diff = 1;
        }
    }

    // If state changed, update the value of old_state,
    // and set a flag that the state has changed.
    if (new_state != old_state)
    {
        encoder_changed = 1;
        old_state = new_state;

        // update values
        if (THRESHOLD_SELECT == LOW) {
            low_thresh += diff;
        }
        else {
            high_thresh += diff;
        }

        // update servo if needed
        if (timer1_running) {
            if (THRESHOLD_SELECT == LOW) {
                update_servo(low_thresh);
            }
            else {
                update_servo(high_thresh);
            }
        }
    }
}

// sense pin changes on port D
ISR(PCINT2_vect)
{
    // read inputs
    uint8_t low_button = PIND & (1 << PD2);
    uint8_t high_button = PIND & (1 << PD3);

    // check if buttons are pressed
    if (low_button == 0)
    {
        THRESHOLD_SELECT = LOW;
        lcd_write_equals(12);
        lcd_write_qmark(3);
        timer1_start();
        update_servo(low_thresh);
    }
    if (high_button == 0)
    {
        THRESHOLD_SELECT = HIGH;
        lcd_write_equals(3);
        lcd_write_qmark(12);
        timer1_start();
        update_servo(high_thresh);
    }

    button_changed = 1;
}