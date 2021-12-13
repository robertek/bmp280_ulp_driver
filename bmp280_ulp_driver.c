/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Robert David <robert.david@posteo.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <math.h>
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"

#include "ulp_bmp280_ulp_driver.h"
#include "bmp280_ulp_driver.h"


extern const uint8_t ulp_bin_start[]
    asm("_binary_ulp_bmp280_ulp_driver_bin_start");
extern const uint8_t ulp_bin_end[]
    asm("_binary_ulp_bmp280_ulp_driver_bin_end");


#define T_SB 7
#define POWER_MODE 1

#define CONFIG(f)  ((T_SB <<5) + ((f) <<2))
#define CTRL_MEAS(t, p) (((t) <<5) + ((p) <<2) + POWER_MODE)

/*
 * BMP280 calibration data read by the ULP
 */
#define t1 ((uint16_t)ulp_t1)
#define t2 ((int16_t) ulp_t2)
#define t3 ((int16_t) ulp_t3)
#define dig_P1 ((uint16_t)ulp_p1)
#define dig_P2 ((int16_t) ulp_p2)
#define dig_P3 ((int16_t) ulp_p3)
#define dig_P4 ((int16_t) ulp_p4)
#define dig_P5 ((int16_t) ulp_p5)
#define dig_P6 ((int16_t) ulp_p6)
#define dig_P7 ((int16_t) ulp_p7)
#define dig_P8 ((int16_t) ulp_p8)
#define dig_P9 ((int16_t) ulp_p9)
#define temp_msb ((uint8_t) ulp_temp_msb)
#define temp_lsb ((uint8_t) ulp_temp_lsb)
#define temp_xlsb ((uint8_t) ulp_temp_xlsb)
#define pres_msb ((uint8_t) ulp_pres_msb)
#define pres_lsb ((uint8_t) ulp_pres_lsb)
#define pres_xlsb ((uint8_t) ulp_pres_xlsb)

int bmp280_ulp_setup( bmp280_ulp_config_t * config )
{
	gpio_num_t gpio_scl, gpio_sda;

	/*
	 * Set the gpio ports
	 */
	if( config && config->gpio_scl) {
		gpio_scl = config->gpio_scl;
	} else {
		gpio_scl = GPIO_NUM_32;
	}

	if( config && config->gpio_sda) {
		gpio_sda =  config->gpio_sda; 
	} else {
		gpio_sda =  GPIO_NUM_33;
	}

	rtc_gpio_init(gpio_scl);
	rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_init(gpio_sda);
	rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY);

	/*
	 * Load the ulp code
	 */
	ESP_ERROR_CHECK(ulp_load_binary(0, ulp_bin_start, 
	    (ulp_bin_end - ulp_bin_start) / sizeof(uint32_t)));

	/*
	 * Set the temp and pres thresholds
	 */
	if(config && config->t_diff) {
		ulp_t_diff = config->t_diff;
	} else {
		ulp_t_diff = DEFAULT_T_DIFF;
	}

	if(config && config->p_diff) {
		ulp_p_diff = config->p_diff;
	} else {
		ulp_p_diff = DEFAULT_P_DIFF;
	}

	/*
	 * Set ulp wakeup period to 1s and set the read period
	 */
	ulp_set_wakeup_period(0, 1000*1000);

	if(config && config->period) {
		ulp_period = config->period;
	} else {
		ulp_period = DEFAULT_PERIOD;
	}

	/*
	 * Set the reg_config
	 */
	if( config && config->filter) {
		ulp_reg_config = CONFIG(config->filter);
	} else {
		ulp_reg_config = CONFIG(DEFAULT_FILTER);
	}

	/*
	 * Set the reg_ctrl
	 */
	if( config ) {
		ulp_reg_ctrl = CTRL_MEAS(
		    config->osrs_t ? config->osrs_t : DEFAULT_OSRS_T,
		    config->osrs_p ? config->osrs_p : DEFAULT_OSRS_P);
	} else {
		ulp_reg_ctrl = CTRL_MEAS(DEFAULT_OSRS_T, DEFAULT_OSRS_P);
	}

	return(0);
}

static int32_t bme280_t_fine;

static int32_t bme280_compensate_T(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)t1<<1))) * ((int32_t)t2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)t1)) * ((adc_T>>4) - ((int32_t)t1))) >> 12) * 
	    ((int32_t)t3)) >> 14;
	bme280_t_fine = var1 + var2;
	T = (bme280_t_fine * 5 + 128) >> 8;
	return T;
}

float bmp280_ulp_get_temp()
{
	uint32_t adc_T =
	    (uint32_t)(((temp_msb << 16) | (temp_lsb << 8) | temp_xlsb) >> 4);

	if (adc_T == 0x80000 || adc_T == 0xfffff) {
		return(0);
	} else {
		return(bme280_compensate_T(adc_T)/100.0);
	}
}

/*
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer
 * bits and 8 fractional bits). Output value of “24674867” represents 
 * 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
float bme280_compensate_P(int32_t adc_P)
{
	float var1, var2, p;

	var1=bme280_t_fine/2.0-64000.0;
	var2=var1*var1*dig_P6/32768.0;
	var2=var2+var1*dig_P5*2;
	var2=var2/4.0+dig_P4*65536.0;
	var1=(dig_P3*var1*var1/524288.0+dig_P2*var1)/524288.0;
	var1=(1.0+var1/32768.0)*dig_P1;

	p=1048576.0-adc_P;
	p=(p-var2/4096.0)*6250.0/var1;
	var1=dig_P9*p*p/2147483648.0;
	var2=p*dig_P8/32768.0;
	p=p+(var1+var2+dig_P7)/16.0;

	return p;
}

float bmp280_ulp_get_pres()
{
	uint32_t adc_P = 
	    (uint32_t)(((pres_msb << 16) | (pres_lsb << 8) | pres_xlsb) >> 4);

	if (adc_P ==0x80000 || adc_P == 0xfffff) {
		return(0);
	} else {
		return(bme280_compensate_P(adc_P)/100.0);
	}
}

void bmp280_ulp_enable()
{
	ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
	ESP_ERROR_CHECK(
	    ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)) );
}
