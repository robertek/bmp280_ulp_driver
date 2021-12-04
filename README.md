# bmp280_ulp_driver

ESP32 ULP driver for BMP280 sensor.
It is plain ulp code and c api for inclusion in other project requiring to
collect BMP280 through ULP coprocessor.
Targeted mainly for battery powered sensors.

## api

## sample code

Include the driver in your code:

    $ cd path/to/my/project
    $ git clone https://github.com/robertek/bmp280_ulp_driver
    $ ls
    bmp280_ulp_driver CMakeLists.txt  main  sdkconfig

Update the CMakeLists to include the driver

    $ cat main/CMakeLists.txt
    idf_component_register(SRCS "bmp280_ulp_example"
                    INCLUDE_DIRS "." "../bmp280_ulp_driver/"
                    REQUIRES soc)
    $ cat CMakeLists.txt
    cmake_minimum_required(VERSION 3.5)
    
    set(EXTRA_COMPONENT_DIRS ./bmp280_ulp_driver)
    
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(bmp280_ulp_example)

You may test it with this example code

    $ cat main/bmp280_ulp_example.c
    #include <stdio.h>
    #include "esp_sleep.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    
    #include "bmp280_ulp_driver.h"
    
    void app_main()
    {
          if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_ULP) {
                  bmp280_ulp_setup( NULL );
          } else {
                  printf("Temp: %.2f C\n", bmp280_ulp_get_temp());
                  printf("Pres: %.2f hPa\n", bmp280_ulp_get_pres());
          }

          bmp280_ulp_enable();
          vTaskDelay(20);
          esp_deep_sleep_start();
    }



## Credits
This code is based on  the https://github.com/xlfe/ulp-i2c

But it is not a fork, because it was heavily rewritten.
