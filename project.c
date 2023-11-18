#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "lcd.h"
#include "ds18b20.h"
#include "buzzer.h"
#include "servo.h"
#include "timer.h"
#include "temp.h"

#define RED PC3
#define GREEN PC4
#define BLUE PC5

#define LOW_THRESH_EEPROM_LOC 0
#define HIGH_THRESH_EEPROM_LOC 1

void change_state(int8_t temp);

enum
{
    NORMAL,
    LOW,
    HIGH,
    BELOW_3,
    ABOVE_3
};

volatile uint8_t new_state, old_state;
volatile char encoder_changed = 0; // Flag for state change
volatile char button_changed = 0;

volatile char THRESHOLD_SELECT;
volatile uint8_t high_thresh;
volatile uint8_t low_thresh;

char STATE;
volatile int current_temp_16;
extern volatile char timer1_running;

int main()
{
    // initialize button pull-ups
    PORTD |= ((1 << PD2) | (1 << PD3));

    // initialize encoder pull-ups
    PORTC |= ((1 << PC1) | (1 << PC2));

    // set ENCODER pin change interrupts (Port C)
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);

    // set button pin change interrupts (Port D)
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);

    // set LED bits to output
    DDRC |= (1 << RED) | (1 << GREEN) | (1 << BLUE);

    // set buzzer bit to output
    DDRB |= (1 << PB5);

    // set servo bit to output
    DDRB |= (1 << PB3);

    // system enable interrupts
    sei();

    // initialize lcd
    lcd_init();

    // Initialize the DS18B20
    if (ds_init() == 0)
    {
        // Sensor not responding
        lcd_errormsg("DSinit no resp");
        while (1)
            ; // stop
    }

    // initialize timer0 for buzzer
    timer0_init();

    // initialize timer1 for 4 second indicator dial
    timer1_init();

    // initialize state of rotary encoder
    uint8_t x = PINC;
    char a = x & (1 << PC1);
    char b = x & (1 << PC2);

    if (!b && !a)
        old_state = 0;
    else if (!b && a)
        old_state = 1;
    else if (b && !a)
        old_state = 2;
    else
        old_state = 3;

    new_state = old_state;

    THRESHOLD_SELECT = LOW;

    // try to read EEPROM
    low_thresh = eeprom_read_byte((void *)LOW_THRESH_EEPROM_LOC);
    high_thresh = eeprom_read_byte((void *)HIGH_THRESH_EEPROM_LOC);

    // make values valid
    check_bounds(&low_thresh);
    check_bounds(&high_thresh);

    // set initial TEMP_STATE
    current_temp_16 = get_temp_F_16();
    int old_temp_16 = -1;

    // if ((current_temp_16 / 16) < low_thresh)
    // {
    //     STATE = LOW;
    //     if ((current_temp_16 / 16) < low_thresh - 3)
    //     {
    //         STATE = BELOW_3;
    //     }
    // }
    // else if ((current_temp_16 / 16) > high_thresh)
    // {
    //     STATE = HIGH;
    //     if ((current_temp_16 / 16) > high_thresh + 3)
    //     {
    //         STATE = ABOVE_3;
    //     }
    // }
    // else
    // {
    //     STATE = NORMAL;
    // }

    // get initial state
    change_state(get_rounded_temp_F(current_temp_16));

    // write splash screen
    lcd_writecommand(1);
    lcd_moveto(0, 0);
    lcd_stringout("EE 109 Project");
    lcd_moveto(1, 0);
    lcd_stringout("Nate Boxer");
    _delay_ms(1000);
    lcd_writecommand(1);

    lcd_moveto(0, 0);
    lcd_stringout("Temp: ");
    lcd_moveto(1, 0);
    lcd_stringout("Low? ");
    lcd_write_thresh_val(low_thresh, 5);
    lcd_moveto(1, 8);
    lcd_stringout("High= ");
    lcd_write_thresh_val(high_thresh, 14);

    // initialize servo
    servo_init(get_rounded_temp_F(current_temp_16));

    /**
     * MAIN PROGRAM LOOP
     */
    while (1)
    {
        // check rotary encoder changes
        if (encoder_changed == 1)
        {
            encoder_changed = 0;

            // check high isn't less than low and vice versa
            if (THRESHOLD_SELECT == LOW && low_thresh > high_thresh)
            {
                low_thresh = high_thresh;
            }
            if (THRESHOLD_SELECT == HIGH && high_thresh < low_thresh)
            {
                high_thresh = low_thresh;
            }

            // write value to screen and EEPROM
            if (THRESHOLD_SELECT == LOW)
            {
                lcd_write_thresh_val(low_thresh, 5);
                eeprom_update_byte((void *)LOW_THRESH_EEPROM_LOC, low_thresh);
            }
            else
            {
                lcd_write_thresh_val(high_thresh, 14);
                eeprom_update_byte((void *)HIGH_THRESH_EEPROM_LOC, high_thresh);
            }
        }

        // read temperature
        current_temp_16 = get_temp_F_16();

        // ignore extraneous temperature results caused from loose connections
        int rounded_temp = get_rounded_temp_F(current_temp_16);
        if (rounded_temp > 120 || rounded_temp < 0) {
            continue;
        }

        // performs next state transition logic based on current temp reading
        change_state(rounded_temp);

        // state output logic for LEDs
        if (STATE == LOW || STATE == BELOW_3)
        {
            // turn on red LED
            PORTC &= ~(1 << RED);
            PORTC |= (1 << GREEN);
            PORTC |= (1 << BLUE);
        }
        else if (STATE == HIGH || STATE == ABOVE_3)
        {
            // turn on blue LED
            PORTC |= (1 << RED);
            PORTC |= (1 << GREEN);
            PORTC &= ~(1 << BLUE);
        }
        else
        {
            // turn on green LED
            PORTC |= (1 << RED);
            PORTC &= ~(1 << GREEN);
            PORTC |= (1 << BLUE);
        }

        // check if temperature has changed and writes new value to display/servo if needed
        if (current_temp_16 != old_temp_16)
        {

            int num = current_temp_16 / 16;
            int dec = (((current_temp_16 % 16) * 10) / 16) % 10;

            char buf[5];
            snprintf(buf, 5, "%d.%d", num, dec);
            lcd_moveto(0, 7);
            lcd_stringout(buf);

            // DISPLAY TO SERVO IF NOT IN THRESHOLD SELECT MODE
            if (!timer1_running) {
                update_servo(get_rounded_temp_F(current_temp_16));
            }

            old_temp_16 = current_temp_16;

            // clear weird bug
            lcd_moveto(0,11);
            lcd_stringout("     ");
        }
    }
    return 0;
}

/**
 * Changes state based on current temperature reading
 */
void change_state(int8_t temp)
{
    if (low_thresh <= temp && temp < high_thresh)
    {
        STATE = NORMAL;
    }
    else if (low_thresh - 3 < temp && temp <= low_thresh)
    {
        STATE = LOW;
    }
    else if (high_thresh <= temp && temp < high_thresh + 3)
    {
        STATE = HIGH;
    }
    else if (temp < low_thresh - 3 && STATE != BELOW_3)
    {
        STATE = BELOW_3;
        // SOUND BUZZER
        play_note();
    }
    else if (temp > high_thresh + 3 && STATE != ABOVE_3)
    {
        STATE = ABOVE_3;
        // SOUND BUZZER
        play_note();
    }
}