// Include necessary headers
#include <stdio.h>
#include "esp_log.h"

// Header guard to prevent multiple inclusion
#ifndef TFLUNA_H
#define TFLUNA_H

#define LUNA_COUNT 4
#define DEF_LUNA_SDA GPIO_NUM_21
#define DEF_LUNA_SCL GPIO_NUM_21


#include "I2Cbus.hpp"


/**
 * @brief lunaHandler to operate multiple ray sensors parallely
 * 
 * instanciate a lunaHandler object with respective parameters
 * and call update() to read and load data into distances array.
 * 
 */
class lunaHandler {
public:

    uint16_t distances[LUNA_COUNT] = {0};

    /**
     * @brief Construct a new luna Handler object
     * 
     * @param SCL assign SCL gpio pin for I2C comm
     * @param SDA assign SDA gpio pin for I2C comm
     * @param addresses pass an array of luna I2c addresses
     * @param count number of addresses in array
     */
    lunaHandler(gpio_num_t SCL = DEF_LUNA_SCL, gpio_num_t SDA = DEF_LUNA_SDA, uint8_t* addresses = DEF_LUNA_ADDRESS, size_t count = LUNA_COUNT);


    /**
     * @brief reads the distance of luna
     * 
     * @param device_address pass the address of luna 
     * @param buffer array to store the data from luna
     */
    void readDistance(uint8_t device_address, uint8_t* buffer);

    /**
     * @brief reads data from luna sensors via I2c bus
     * and updates data into distances array
     * 
     */
    void update();

private:
    i2cbus::I2C i2c; // I2C interface object
    
    uint8_t addresses[LUNA_COUNT];
    static uint8_t DEF_LUNA_ADDRESS[4];
};

#endif // TFLUNA_H
