#include <stdio.h>
#include <unistd.h>

#include <driver/gpio.h>
#include <driver/ledc.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_srvs/srv/set_bool.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LED_R 14
#define LED_G 12
#define LED_B 13

rcl_publisher_t publisher;
rcl_init_options_t init_ops;
size_t domain_id = 41;
std_msgs__msg__Int32MultiArray msg;

void setColor(int red, int green, int blue)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, red);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, green);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, blue);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    int32_t pwm_values[3] = {
        ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0),
        ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1),
        ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2)
    };
    msg.data.data = pwm_values;
    msg.data.size = 3;
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
}

void service_callback(const void * req, void * res)
{
    std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) req;
    std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) res;
    if (req_in->data)
    {
        setColor(0, 255, 0);
    }
    else
    {
        setColor(255, 0, 0);
    }
    res_in->success = true;
}

void appMain(void * arg)
{
    std_srvs__srv__SetBool_Request req;
    std_srvs__srv__SetBool_Response res;
    std_msgs__msg__Int32MultiArray__init(&msg);
    std_srvs__srv__SetBool_Request__init(&req);
    std_srvs__srv__SetBool_Response__init(&res);

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };

    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel[3] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = LED_R,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = LED_G,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 0,
            .gpio_num   = LED_B,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        }
    };

    for (int ch = 0; ch < 3; ch++) 
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;
    // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    init_ops = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_ops, allocator));

    RCCHECK(rcl_init_options_set_domain_id(&init_ops, domain_id));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_ops, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "led", "esp32", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/led_state"));

    rcl_service_t service;
    RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/set_state"));

    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
    
    rclc_executor_spin(&executor);

    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rcl_service_fini(&service, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));

    std_msgs__msg__Int32MultiArray__fini(&msg);
    std_srvs__srv__SetBool_Request__fini(&req);
    std_srvs__srv__SetBool_Response__fini(&res);

    vTaskDelete(NULL);
}