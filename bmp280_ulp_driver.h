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

/*
 * BMP280 Config options.
 *
 * OSRS_P = 1 # 16 Bit ultra low power
 * OSRS_P = 2 # 17 Bit low power
 * OSRS_P = 3 # 18 Bit standard resolution
 * OSRS_P = 4 # 19 Bit high resolution
 * OSRS_P = 5 # 20 Bit ultra high resolution
 *
 * OSRS_T = 0 # skipped
 * OSRS_T = 1 # 16 Bit
 * OSRS_T = 2 # 17 Bit
 * OSRS_T = 3 # 18 Bit
 * OSRS_T = 4 # 19 Bit
 * OSRS_T = 5 # 20 Bit
 *
 * FILTER = 0 #
 * FILTER = 1 #
 * FILTER = 2 #
 * FILTER = 3 #
 * FILTER = 4 #
 * FILTER = 5 #
 * FILTER = 6 #
 * FILTER = 7 #
 */

#define DEFAULT_OSRS_P 1
#define DEFAULT_OSRS_T 1
#define DEFAULT_FILTER 0

#define DEFAULT_T_DIFF 20
#define DEFAULT_P_DIFF 10

#define DEFAULT_PERIOD 5

typedef struct bmp280_ulp_config {
	int osrs_p;
	int osrs_t;
	int filter;
	int t_diff;
	int p_diff;
	int period;
	gpio_num_t gpio_scl;
	gpio_num_t gpio_sda;
} bmp280_ulp_config_t;

int bmp280_ulp_setup(bmp280_ulp_config_t *);
float bmp280_ulp_get_temp();
float bmp280_ulp_get_pres();
void bmp280_ulp_enable();
