#include <stdio.h>

#include <driver/uart.h>

#include "lsaHandler.hpp"

#define UART_BUFFER_SIZE 1024


lsaHandler::lsaHandler(gpio_num_t TX, gpio_num_t RX, gpio_num_t en, gpio_num_t trigger) : en(en){

    // init uart
    const uart_port_t uart_num = LSA_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

// Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TX, RX, GPIO_NUM_NC, GPIO_NUM_NC);
    uart_driver_install(uart_num, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);

    gpio_config_t enConf {
        .pin_bit_mask = (uint64_t)(1 << en),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config_t triggerConf {
        .pin_bit_mask = (uint64_t)(1 << trigger),
        .mode = GPIO_MODE_INPUT,
        // .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    gpio_config(&enConf);
    gpio_config(&triggerConf);

    gpio_install_isr_service( ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED);
    gpio_isr_handler_add(trigger, lsaTriggerIsrCallback,  this);
    gpio_intr_enable(trigger);


}


void lsaHandler::update(){
    const uart_port_t uart_num = LSA_UART_PORT;
    gpio_set_level(en, 0);
    const int rxBytes = uart_read_bytes(uart_num, &value, 1, pdMS_TO_TICKS(1000));
    gpio_set_level(en, 1);
}


void lsaHandler::lsaTriggerIsrCallback(void *ptr){
    lsaHandler* handler = (lsaHandler*) ptr;
    handler->trigger = true;
}