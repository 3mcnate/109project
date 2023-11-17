#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "temp.h"
#include "ds18b20.h"

/**
 * @return temperature in farenheit times 16 (must divide by 16 to get actual value)
 */
int get_temp_F_16(void)
{
    // start conversion
    ds_convert();

    unsigned char t[2];

    while (1)
    {
        if (ds_temp(t)) // True if conversion complete
        {
            // value is scaled by 16.
            int16_t value = t[1];
            value = value << 8;
            value |= t[0];

            value = (value * 9) / 5 + (16 * 32);
            return value;
        }
    }
}

/**
 * @brief returns temperature rounded to the nearest integer
 * @param[in] temp temperature in Farenheit, times 16
 */
uint8_t get_rounded_temp_F(int temp)
{
    uint8_t val = temp / 16;
    if (temp % 16 >= 8)
    {
        val++;
    }
    return val;
}