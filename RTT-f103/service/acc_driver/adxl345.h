#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>

#define SENSOR_ACCEL_INT_SRC_ERR        0
#define SENSOR_ACCEL_INT_SRC_ACT        1
#define SENSOR_ACCEL_INT_SRC_INACT      2


/* 中断回调函数 */
typedef void (*adxl345_int_cb)(void *args);


typedef struct  {
    struct rt_i2c_bus_device *i2c;  //i2c总线设备
    adxl345_int_cb   int_cb;        //回调函数
    rt_mutex_t       lock;          //数据操作锁
}adxl345_device_s; /* adxl345设备定义 */


/* 设备初始化 */
adxl345_device_s *adxl345_init(const char *i2c_bus_name);

/* 读取当前加速度原始值 */
rt_err_t adxl345_read(adxl345_device_s *pdev, rt_int16_t *x, rt_int16_t *y, rt_int16_t *z);

rt_err_t sensor_accel_init(void);

rt_err_t sensor_accel_read(int16_t *x, int16_t *y, int16_t *z);

rt_err_t task_acc_start(uint8_t priority);

void acc_task_entry(void *parameter);
