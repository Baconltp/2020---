#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <board.h>
#include <stdio.h>

#define LED0_PIN     GET_PIN(C, 13)

rt_err_t task_led_start(uint8_t priority);

void led_task_entry(void *parameter);
