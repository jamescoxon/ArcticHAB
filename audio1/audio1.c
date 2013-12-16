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


/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#define     __I     volatile const            /*!< defines 'read only' permissions      */
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/dac.h>
#include <stdio.h>

#include <arm_math.h>
#include "arm_const_structs.h"

void delay(uint32_t count);
void delay_ms(uint32_t ms);
void delay_ns(uint32_t ns);
void send_string(char * string);
void send_newline(void);
int read_adc(void);
void generate_graph(int raw_data[256]);

//DAC
void dac_setup(void);
void set_dac_level(int power);

#define DELAY_1_MS (uint32_t)(30925)
#define DELAY_MAX_MS (0xFFFFFFFF / DELAY_1_MS)
#define DELAY_1_NS (uint32_t)(31)
#define DELAY_MAX_NS (0xFFFFFFFF / DELAY_1_NS)

/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    
	/* Enable GPIOD clock. */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    
	/* Enable clocks for USART2. */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);
}

static void gpio_setup(void)
{
	/* Set GPIO12-15 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    
    /* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    
	/* Setup USART2 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
}

static void usart_setup(void)
{
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    
	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void adc_setup(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_3CYC);
    uint8_t channels[] = {ADC_CHANNEL1};
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
    adc_power_on(ADC1);
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

void delay_ns(uint32_t ns) {
    if(ns > DELAY_MAX_NS) {
        uint8_t i;
        for(i=0; i<(ns / DELAY_MAX_NS) - 1; i++)
            delay(0xFFFFFFFF);
        delay(ns % DELAY_MAX_NS);
    } else {
        delay(ns * DELAY_1_NS);
    }
}

void send_newline(void){
    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');
}

void send_string(char * string){
    
    int i = 0;
    
	while ( string[i] != '\0')
	{
		usart_send_blocking(USART2, string[i]); /* USART2: Send byte. */
        i++;
	}
    
    
}

int read_adc(void){
	adc_start_conversion_regular(ADC1);
	while (! adc_eoc(ADC1));
	int reg16;
	reg16 = adc_read_regular(ADC1);
	return reg16;
}

void generate_graph(int raw_data[256]){

    //First define our 10 blocks, each will be a printed string of 20 chars long
    //0
    char line0[40] = "                                       ";
    //1
    char line1[40] = "                                       ";
    //2
    char line2[40] = "                                       ";
    //3
    char line3[40] = "                                       ";
    //4
    char line4[40] = "                                       ";
    //5
    char line5[40] = "                                       ";
    //6
    char line6[40] = "                                       ";
    //7
    char line7[40] = "                                       ";
    //8
    char line8[40] = "                                       ";
    //9
    char line9[40] = "                                       ";
    
    //Now we work through our sample and look at the value and put the corresponding point in the right place.
    
    int gap_between_samples = 256 / 40;
    
    int scale_max = 1024, scale_min = 768;
    int scaled_gap = (scale_max - scale_min) / 10;
    int scale_mid_point = (scale_min + (scale_max - scale_min) / 2);
    int i;
    uint8_t pos = 0;
    
    //send_string("Start graph");
    //send_newline();
    for(i=0; i < 255; (i = i + gap_between_samples)){
        
        if(raw_data[i] < (scale_mid_point - (scaled_gap * 4)))                                                                  {line0[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point - (scaled_gap * 4)))   &   (raw_data[i] < (scale_mid_point - scaled_gap * 3))) {line1[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point - (scaled_gap * 3)))   &   (raw_data[i] < (scale_mid_point - scaled_gap * 2))) {line2[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point - (scaled_gap * 2)))   &   (raw_data[i] < (scale_mid_point - scaled_gap)))     {line3[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point - scaled_gap))         &   (raw_data[i] < (scale_mid_point)))                  {line4[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point))                      &   (raw_data[i] < (scale_mid_point + scaled_gap)))     {line5[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point + scaled_gap))         &   (raw_data[i] < (scale_mid_point + scaled_gap * 2))) {line6[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point + (scaled_gap * 2)))   &   (raw_data[i] < (scale_mid_point + scaled_gap * 3))) {line7[pos] = '*';}
        else if((raw_data[i] >= (scale_mid_point + (scaled_gap * 3)))   &   (raw_data[i] < (scale_mid_point + scaled_gap * 4))) {line8[pos] = '*';}
        else if(raw_data[i] >= (scale_mid_point + (scaled_gap * 4)))                                                            {line9[pos] = '*';}
        //else {};

        pos++;
    }
    
    send_newline();
    send_newline();
    send_newline();
    send_newline();
    send_newline();
    
    
    send_string("Print Graph");
    send_newline();
    
    
    //Now print the data
    
    send_string(line9);
    send_newline();
    send_string(line8);
    send_newline();
    send_string(line7);
    send_newline();
    send_string(line6);
    send_newline();
    send_string(line5);
    send_newline();
    send_string(line4);
    send_newline();
    send_string(line3);
    send_newline();
    send_string(line2);
    send_newline();
    send_string(line1);
    send_newline();
    send_string(line0);
    send_newline();
    

    send_newline();
    send_newline();
    send_newline();
    send_newline();
    send_newline();
    send_newline();
    send_newline();
    //send_newline();
    //send_newline();
    
    
}

int main(void)
{
    int testInput_f32_10khz[256];
    char superbuffer [80]; //Telem string buffer
    
	clock_setup();
	gpio_setup();
    usart_setup();
    adc_setup();
    dac_setup();
    
    gpio_set(GPIOD, GPIO12);

    set_dac_level(2750);
    
    //send_string("Starting data collection");
    //send_newline();
	/* Blink the LED (PC8) on the board. */
	while (1) {
        
        //send_string("ADC");
        //send_newline();

        int i;
        for(i=0; i<255; i++){
            testInput_f32_10khz[i] = read_adc();
            //send_string(".");
        }
        //send_newline();
        
        //send_string("Collected data, graphing..."); send_newline();
        //generate_graph(testInput_f32_10khz);
        
        
        for(i=0; i<255; (i=i+6)){
            sprintf (superbuffer, "%d", testInput_f32_10khz[i]);
            send_string(superbuffer);
            send_newline();
        }
        //send_string("Completed");
        //send_newline();

        delay_ms(1000);
        
    }

	return 0;
}



