#ifndef VARS_H
#define VARS_H
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "lcd.h"
#include "ds18b20.h"

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
volatile char diff = 0;

char STATE;
volatile char THRESHOLD_SELECT;
volatile char button_changed = 0;
uint8_t high_thresh;
uint8_t low_thresh;

#endif