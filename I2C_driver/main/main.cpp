#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "lunaHandler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern "C" void app_main(void)
{   

    lunaHandler handle;

    while (1) {
        handle.update();

        ESP_LOGI("main", " distances %d %d %d %d", 
            handle.distances[0],
            handle.distances[1],
            handle.distances[2],
            handle.distances[3]
            );

        vTaskDelay(pdMS_TO_TICKS(100));
    }
       
}
