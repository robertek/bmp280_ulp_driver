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
