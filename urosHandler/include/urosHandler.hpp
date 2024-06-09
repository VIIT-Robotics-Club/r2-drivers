
#ifndef UROS_HANDLER_HPP
#define UROS_HANDLER_HPP

#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "urosElement.hpp"


/**
 * @brief urosHandler to manage micro ros initialization
 * 
 */
class urosHandler {

public:

    /**
     * @brief Construct a new uros Handler object
     * 
     * @param node_name ros2 node name
     */
    urosHandler(std::string node_name = "uros_node");

    /**
     * @brief create a rclc executor on a new thread   
     * 
     * @param elements vector of urosElement pointer to run on these threaded executor
     * @param CPUID  cpuid to run executor thread 
     */
    void addThreadExecutor(std::vector<urosElement*> elements, int CPUID = PRO_CPU_NUM);


private:


    static QueueHandle_t m_urosAccess;
    static void executorTask(void* param);

    rcl_allocator_t allocator;
    rclc_support_t support;
    rmw_init_options_t* rmw_options;
    rcl_node_t node;
};



#endif UROS_HANDLER_HPP
