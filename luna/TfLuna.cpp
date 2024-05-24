#include <stdio.h>
#include "esp_log.h"
#include <TfLuna.hpp>
#include <I2Cbus.hpp>

// Constructor: Initialize TFLuna object with the provided I2C interface
Tfluna::Tfluna(i2cbus::I2C& i2c_interface) : i2c(i2c_interface) {}

// Initialize the TFLuna sensor with the given SDA and SCL pins
void Tfluna::begin(gpio_num_t sda_pin, gpio_num_t scl_pin) {
    i2c.begin(sda_pin, scl_pin); // Initialize I2C communication
}

// Set the timeout for I2C communication
void Tfluna::setTimeout(uint32_t timeout_ms) {
    i2c.setTimeout(timeout_ms); // Set the I2C timeout
}

// Scan for I2C devices on the bus
void Tfluna::scanner() {
    i2c.scanner(); // Scan for I2C devices
}

// Write a byte to the TFLuna device at the specified address
void Tfluna::writeByte(uint8_t device_address) {
    // Write 0x01 followed by 0x00 to the specified device address
    i2c.writeByte(device_address, 0x25, 0x01);
    i2c.writeByte(device_address, 0x25, 0x00);
}

// Read distance data from the TFLuna device at the specified address into the provided buffer
void Tfluna::readDistance(uint8_t device_address, uint8_t* buffer) {
    // Read 1 byte from register 0x00 of the device into the buffer
    i2c.readBytes(device_address, 0x00, 1, buffer);
}

// Close the I2C communication
void Tfluna::close() {
    i2c.close(); // Close I2C communication
}
