/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

//Command to upload via DFU
//dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D blinky.bin

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/dac.h>

void delay(uint32_t count);
void delay_ms(uint32_t ms);

#define DELAY_1_MS (uint32_t)(24740)
#define DELAY_MAX_MS (0xFFFFFFFF / DELAY_1_MS)

//DAC
void dac_setup(void);
void set_dac_level(int power);

//RTTY
void rtty_txbit (int bit);
void rtty_txbyte (char c);
void rtty_txstring (char * string);

/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    
	/* Enable GPIOD clock. */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
}

static void gpio_setup(void)
{
	/* Set GPIO12-15 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

//https://github.com/pcbwriter/pcbwriter/blob/master/firmware/dac.c
void dac_setup(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_DACEN);
    dac_enable(CHANNEL_1);
}

void set_dac_level(int power)
{
    dac_load_data_buffer_single(power, RIGHT12, CHANNEL_1);
}

//Adding in RTTY
// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{
    
	/* Simple function to sent a char at a time to
     ** rtty_txbyte function.
     ** NB Each char is one byte (8 Bits)
     */
	char c;
	c = *string++;
	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
}

void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to
     ** rtty_txbit function.
     ** NB The bits are sent Least Significant Bit first
     **
     ** All chars should be preceded with a 0 and
     ** proceded with a 1. 0 = Start bit; 1 = Stop bit
     **
     ** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
     */
	int i;
	rtty_txbit (0); // Start bit
	// Send bits for for char LSB first
	for (i=0;i<8;i++)
	{
		if (c & 1) rtty_txbit(1);
        else rtty_txbit(0);
		c = c >> 1;
	}
	rtty_txbit (1); // Stop bit
    rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
    if (bit)
    {
        // high
        gpio_set(GPIOD, GPIO12);	/* LED on */
        set_dac_level(2500);
    }
    else
    {
        // low
        gpio_clear(GPIOD, GPIO12);  /* LED off */
        set_dac_level(3000);
    }
    delay_ms(19); // 10000 = 100 BAUD 20150
    
}

void delay(uint32_t count) {
    uint32_t i;
    for(i=0; i<count; i++)
        __asm__("nop");
}

void delay_ms(uint32_t ms) {
    if(ms > DELAY_MAX_MS) {
        uint8_t i;
        for(i=0; i<(ms / DELAY_MAX_MS) - 1; i++)
            delay(0xFFFFFFFF);
        delay(ms % DELAY_MAX_MS);
    } else {
        delay(ms * DELAY_1_MS);
    }
}

int main(void)
{
	clock_setup();
	gpio_setup();
    dac_setup();
    
	gpio_set(GPIOD, GPIO12 | GPIO13);
    delay_ms(1000);
    gpio_clear(GPIOD, GPIO12 | GPIO13);
    delay_ms(1000);

	

	/* Blink the LED (PC8) on the board. */
	while (1) {

        rtty_txstring("TEST TEST TEST");
    }

	return 0;
}



