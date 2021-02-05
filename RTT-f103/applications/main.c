/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-04     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <adxl345.h>
#include <led.h>

//#define acc

void user_app_init()
{
#ifdef acc
    sensor_accel_init();
#endif
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
}

int main(void)
{
    user_app_init();

#ifdef acc
    if(RT_EOK == task_acc_start(15))
    {
        rt_kprintf("[task] acc start success.\r\n");
    }
#endif
    if(RT_EOK == task_led_start(16))
    {
        rt_kprintf("[task] led start success.\r\n");
    }
}
