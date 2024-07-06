#ifndef LSA_HANDLER_HPP
#define LSA_HANDLER_HPP


#include <driver/gpio.h>

#define LSA_UART_PORT UART_NUM_1

class lsaHandler {

public:

    lsaHandler(gpio_num_t TX, gpio_num_t RX, gpio_num_t EN, gpio_num_t trigger);

    void update();

    bool trigger = false;
    uint8_t value = 0;

private:

    gpio_num_t en;
    gpio_isr_handle_t isrHandle;
    static void lsaTriggerIsrCallback(void* ptr); 
};

#endif //  LSA_HANDLER_HPP