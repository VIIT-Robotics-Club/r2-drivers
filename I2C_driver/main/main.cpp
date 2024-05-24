#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "TfLuna.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define addr 0x10
#define addr1 0x13
#define addr2 0x14
#define addr3 0x12
extern "C" void app_main(void)
{   
     
    i2cbus::I2C i2c0(I2C_NUM_0);  // Create an I2C object using I2C_NUM_0
    Tfluna tfluna(i2c0); 
    tfluna.begin(GPIO_NUM_21, GPIO_NUM_22);
    tfluna.setTimeout(10);
    tfluna.scanner();
    
    uint8_t BUFFER[1] ;
    uint8_t BUFFER1[1] ;
    uint8_t BUFFER2[1] ;
    uint8_t BUFFER3[1] ;
    uint8_t dataArray[4][1]; // Array to store BUFFER and BUFFER1 values
    while (1) {
        tfluna.readDistance(addr,BUFFER);
        // ESP_LOGI("MAIN", "%d", BUFFER[0]);
        dataArray[0][0] = BUFFER[0]; // Store BUFFER value in dataArray
        tfluna.readDistance(addr1,BUFFER1);
        // ESP_LOGI("MAIN", "data 2:""%d", BUFFER1[0]);
        dataArray[1][0] = BUFFER1[0]; // Store BUFFER1 value in dataArray
        // Print the values of BUFFER and BUFFER1
        tfluna.readDistance(addr2,BUFFER2);
        // ESP_LOGI("MAIN", "%d", BUFFER[0]);
        dataArray[2][0] = BUFFER2[0]; // Store BUFFER value in dataArray
        tfluna.readDistance(addr3,BUFFER3);
        // ESP_LOGI("MAIN", "%d", BUFFER[0]);
        dataArray[3][0] = BUFFER3[0]; // Store BUFFER value in dataArray
        ESP_LOGI("MAIN", "data 1: %d", dataArray[0][0]);
        vTaskDelay(pdMS_TO_TICKS(500)); // Add a delay of 1000 milliseconds
        ESP_LOGI("MAIN", "data 2: %d", dataArray[1][0]);
        vTaskDelay(pdMS_TO_TICKS(500)); // Add a delay of 1000 milliseconds
        ESP_LOGI("MAIN", "data 3: %d", dataArray[2][0]);
        vTaskDelay(pdMS_TO_TICKS(500)); // Add a delay of 1000 milliseconds
        ESP_LOGI("MAIN", "data 4: %d", dataArray[3][0]);
    }
   
    tfluna.close();
    
}
