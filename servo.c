#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "servo.h"

#define OCR2A_MAX 35
#define OCR2A_MIN 12

void servo_init(uint8_t init_temp)
{
    TCCR2A |= ((1 << WGM21) | (1 << WGM20));
    TCCR2A |= (1 << COM2A1);

    calc_OCR2A(init_temp);

    // set prescalar to 1024, starts PWM
    TCCR2B |= (0b111 << CS20);
}

void update_servo(uint8_t temp)
{
    calc_OCR2A(temp);
}

/**
 * @brief sets OCR2A value for PWM pulse width based on temperature
 */
void calc_OCR2A(uint8_t temp)
{
    // for 0.75ms: OCR2A = 12
    // for 2.25ms: OCR2A = 35
    int t = (((40 - temp) * (OCR2A_MAX - OCR2A_MIN)) / 60) + OCR2A_MAX;

    if (t < OCR2A_MIN)
    {
        OCR2A = OCR2A_MIN;
    }
    else if (t > OCR2A_MAX)
    {
        OCR2A = OCR2A_MAX;
    }
    else
    {
        OCR2A = t;
    }
}