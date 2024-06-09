#include "urosHandler.hpp"


#include <uros_network_interfaces.h>
#include <rmw_microros/rmw_microros.h>
#include <freertos/task.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


QueueHandle_t urosHandler::m_urosAccess;

bool isUrosNetworkInit = false;

urosHandler::urosHandler(std::string nodeName){

    // create mutex for acccess to micro ros methods
    if(!m_urosAccess) m_urosAccess = xSemaphoreCreateMutex();

    if(!isUrosNetworkInit) {
        ESP_ERROR_CHECK(uros_network_interface_initialize());
        isUrosNetworkInit = true;
    };

    allocator = rcl_get_default_allocator();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	

    bool waitAutoDiscovery = true;
    while (waitAutoDiscovery)
    {   
        rmw_ret_t ret = rmw_uros_discover_agent(rmw_options);
        if(ret == RMW_RET_OK) waitAutoDiscovery = false;
        else if(ret == RMW_RET_TIMEOUT) waitAutoDiscovery = true;
        else vTaskDelete(NULL);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    rmw_uros_sync_session(1000);

	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, nodeName.c_str(), "", &support));

}

void urosHandler::addThreadExecutor(std::vector<urosElement*> elements, int CPUID){

    std::vector<urosElement*>* elems = new std::vector<urosElement*>(elements);

    elems->at(0)->node = &node;
    elems->at(0)->support = &support;
    elems->at(0)->alloc = &allocator;

    TaskHandle_t handle;
    xTaskCreatePinnedToCore(executorTask, "uros-exec", 4096, elems, 2, &handle, CPUID);
}



void urosHandler::executorTask(void *param){
    std::vector<urosElement*>* elems = (std::vector<urosElement*>*)param;

    rclc_support_t* support = elems->at(0)->support;
    rcl_allocator_t* alloc = elems->at(0)->alloc;
    rcl_node_t* node = elems->at(0)->node;

    
    // hold mutex while calling uros methods
    xSemaphoreTake(m_urosAccess, portMAX_DELAY);
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support->context, 2, alloc));

    for(auto* elem : *elems){
        elem->support = support;
        elem->exec = &executor;
        elem->alloc = alloc;
        elem->node = node;

        elem->init();
    } 

    xSemaphoreGive(m_urosAccess);
    
    rclc_executor_spin(&executor);
    vTaskDelete(NULL);
}
