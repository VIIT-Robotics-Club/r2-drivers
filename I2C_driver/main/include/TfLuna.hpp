// Include necessary headers
#include <stdio.h>
#include "esp_log.h"

// Header guard to prevent multiple inclusion
#ifndef TFLUNA_H
#define TFLUNA_H

// Include the I2Cbus.hpp header
#include "I2Cbus.hpp"

// Define the Tfluna class
class Tfluna {
public:
    // Constructor: Initialize TFLuna object with the provided I2C interface
    Tfluna(i2cbus::I2C& i2c_interface);

    // Initialize the TFLuna sensor with the given SDA and SCL pins
    void begin(gpio_num_t sda_pin, gpio_num_t scl_pin);

    // Set the timeout for I2C communication
    void setTimeout(uint32_t timeout_ms);

    // Scan for I2C devices on the bus
    void scanner();

    // Read distance data from the TFLuna device at the specified address into the provided buffer
    void readDistance(uint8_t device_address, uint8_t* buffer);

    // Write a byte to the TFLuna device at the specified address
    void writeByte(uint8_t device_address);

    // Close the I2C communication
    void close();

private:
    i2cbus::I2C i2c; // I2C interface object
};

#endif // TFLUNA_H
