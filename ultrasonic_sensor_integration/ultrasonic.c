#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
std_msgs__msg__Int32 msg;
static uint trig_pin = 2;
static uint echo_pin = 3;
rcl_publisher_t publilsher_;
rcl_timer_t timer_;
rclc_support_t support_;
rcl_allocator_t allocator_;
rcl_node_t node_;
rclc_executor_t executor_;

int gpio_config(int tp, int ep)
{
    gpio_init(tp);
    gpio_init(ep);
    gpio_set_dir(tp,GPIO_OUT);
    gpio_set_dir(ep,GPIO_IN);
    return 0;
}
void getPulse(uint tp,uint ep,uint64_t *timeus)
{
    gpio_put(tp,0);
    sleep_us(5);
    gpio_put(tp,1);
    sleep_us(10);
    gpio_put(tp,0);
    while(gpio_get(ep)==0){}
    uint64_t echo_start = time_us_64();
    uint64_t time = echo_start;
    while(gpio_get(ep)==1)
    {
        time = time_us_64();
    }
    *timeus = time-echo_start;
}
void getCm(int tp, int ep,int * distance)
{
    uint64_t timeus;
    getPulse(tp,ep,&timeus);
    *distance = timeus/58;
}
void timer_callback(rcl_timer_t * timer,int64_t last_call_time)
{
    int distance;
    getCm(trig_pin,echo_pin,&distance);
    msg.data = distance;
    rcl_ret_t ret=rcl_publish(&publilsher_,&msg,NULL);
}
int main()
{
    allocator_ = rcl_get_default_allocator();
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    gpio_init(trig_pin);
    gpio_init(echo_pin);
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    gpio_config(trig_pin,echo_pin);
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms,attempts);
    if(ret != RCL_RET_OK)
    {
        return ret;
    }
    rclc_support_init(&support_,0,NULL,&allocator_);
    rclc_node_init_default(&node_,"ultrasonic_node","",&support_);
    rclc_publisher_init_default(&publilsher_,&node_,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int32),"pico_publisher");
    rclc_timer_init_default(
        &timer_,
        &support_,
        RCL_MS_TO_NS(1000),
        timer_callback);
    rclc_executor_init(&executor_,&support_.context,1,&allocator_);
    rclc_executor_add_timer(&executor_,&timer_);
    msg.data = 0;
    while(true)
    {
        rclc_executor_spin_some(&executor_,RCL_MS_TO_NS(1000));
    }
    return 0;
}