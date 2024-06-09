#include <stdio.h>

#include <lunaHandler.hpp>
#include <I2Cbus.hpp>
#include <cstring>


lunaHandler::lunaHandler(gpio_num_t SCL, gpio_num_t SDA, uint8_t* inAddr, size_t count) : count(count), i2c(I2C_NUM_0)  {
    memcpy(addresses, inAddr, 2 * count);
    i2c.begin(SDA, SCL);
    i2c.setTimeout(10);
    i2c.scanner();

    // write enable byte
    // for(int i = 0; i < LUNA_COUNT; i++){
    //     i2c.writeByte(addresses[i], 0x25, 0x01);
    // }

}

// Read distance data from the TFLuna device at the specified address into the provided buffer
void lunaHandler::readDistance(uint8_t device_address, uint8_t* buffer) {
    // Read 1 byte from register 0x00 of the device into the buffer
    i2c.readBytes(device_address, 0x00, 2, buffer);
}

void lunaHandler::update(){
    uint8_t buffer[2];

    for(int i = 0; i < count; i++){
        i2c.readBytes(addresses[i], 0x00, 2, buffer);
        distances[i] = buffer[1] << 8 | buffer[0];  //flip the LSB and MSB
    };

}
