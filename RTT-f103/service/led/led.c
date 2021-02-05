#include <led.h>

rt_err_t task_led_start(uint8_t priority)
{
    rt_thread_t tid = RT_NULL;

    tid = rt_thread_create("led",
                            led_task_entry, RT_NULL,
                            256,priority, 10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        return RT_EOK;
    }
    return -RT_ERROR;
}

void led_task_entry(void *parameter)
{
    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
