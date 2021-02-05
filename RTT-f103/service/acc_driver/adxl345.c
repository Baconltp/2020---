#include <adxl345.h>

/******************************************************************************/
/******************************** ADXL345 *************************************/
/******************************************************************************/

/* I2C address of the device */
#define ADXL345_ADDRESS     0x53

/* ADXL345 Register Map */
#define ADXL345_DEVID           0x00 // R   Device ID.
#define ADXL345_THRESH_TAP      0x1D // R/W Tap threshold.
#define ADXL345_OFSX            0x1E // R/W X-axis offset.
#define ADXL345_OFSY            0x1F // R/W Y-axis offset.
#define ADXL345_OFSZ            0x20 // R/W Z-axis offset.
#define ADXL345_DUR             0x21 // R/W Tap duration.
#define ADXL345_LATENT          0x22 // R/W Tap latency.
#define ADXL345_WINDOW          0x23 // R/W Tap window.
#define ADXL345_THRESH_ACT      0x24 // R/W Activity threshold.
#define ADXL345_THRESH_INACT    0x25 // R/W Inactivity threshold.
#define ADXL345_TIME_INACT      0x26 // R/W Inactivity time.
#define ADXL345_ACT_INACT_CTL   0x27 // R/W Axis enable control for activity
// and inactivity detection.
#define ADXL345_THRESH_FF       0x28 // R/W Free-fall threshold.
#define ADXL345_TIME_FF         0x29 // R/W Free-fall time.
#define ADXL345_TAP_AXES        0x2A // R/W Axis control for tap/double tap.
#define ADXL345_ACT_TAP_STATUS  0x2B // R   Source of tap/double tap.
#define ADXL345_BW_RATE         0x2C // R/W Data rate and power mode control.
#define ADXL345_POWER_CTL       0x2D // R/W Power saving features control.
#define ADXL345_INT_ENABLE      0x2E // R/W Interrupt enable control.
#define ADXL345_INT_MAP         0x2F // R/W Interrupt mapping control.
#define ADXL345_INT_SOURCE      0x30 // R   Source of interrupts.
#define ADXL345_DATA_FORMAT     0x31 // R/W Data format control.
#define ADXL345_DATAX0          0x32 // R   X-Axis Data 0.
#define ADXL345_DATAX1          0x33 // R   X-Axis Data 1.
#define ADXL345_DATAY0          0x34 // R   Y-Axis Data 0.
#define ADXL345_DATAY1          0x35 // R   Y-Axis Data 1.
#define ADXL345_DATAZ0          0x36 // R   Z-Axis Data 0.
#define ADXL345_DATAZ1          0x37 // R   Z-Axis Data 1.
#define ADXL345_FIFO_CTL        0x38 // R/W FIFO control.
#define ADXL345_FIFO_STATUS     0x39 // R   FIFO status.

/* ADXL345_ACT_INACT_CTL definition */
#define ADXL345_ACT_ACDC        (1 << 7)
#define ADXL345_ACT_X_EN        (1 << 6)
#define ADXL345_ACT_Y_EN        (1 << 5)
#define ADXL345_ACT_Z_EN        (1 << 4)
#define ADXL345_INACT_ACDC      (1 << 3)
#define ADXL345_INACT_X_EN      (1 << 2)
#define ADXL345_INACT_Y_EN      (1 << 1)
#define ADXL345_INACT_Z_EN      (1 << 0)

/* ADXL345_TAP_AXES definition */
#define ADXL345_SUPPRESS        (1 << 3)
#define ADXL345_TAP_X_EN        (1 << 2)
#define ADXL345_TAP_Y_EN        (1 << 1)
#define ADXL345_TAP_Z_EN        (1 << 0)

/* ADXL345_ACT_TAP_STATUS definition */
#define ADXL345_ACT_X_SRC       (1 << 6)
#define ADXL345_ACT_Y_SRC       (1 << 5)
#define ADXL345_ACT_Z_SRC       (1 << 4)
#define ADXL345_ASLEEP          (1 << 3)
#define ADXL345_TAP_X_SRC       (1 << 2)
#define ADXL345_TAP_Y_SRC       (1 << 1)
#define ADXL345_TAP_Z_SRC       (1 << 0)

/* ADXL345_BW_RATE definition */
#define ADXL345_LOW_POWER       (1 << 4)
#define ADXL345_RATE(x)         ((x) & 0xF)

/* ADXL345_POWER_CTL definition */
#define ADXL345_PCTL_LINK       (1 << 5)
#define ADXL345_PCTL_AUTO_SLEEP (1 << 4)
#define ADXL345_PCTL_MEASURE    (1 << 3)
#define ADXL345_PCTL_SLEEP      (1 << 2)
#define ADXL345_PCTL_WAKEUP(x)  ((x) & 0x3)

/* ADXL345_INT_ENABLE / ADXL345_INT_MAP / ADXL345_INT_SOURCE definition */
#define ADXL345_DATA_READY      (1 << 7)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_OVERRUN         (1 << 0)

/* ADXL345_DATA_FORMAT definition */
#define ADXL345_SELF_TEST       (1 << 7)
#define ADXL345_SPI             (1 << 6)
#define ADXL345_INT_INVERT      (1 << 5)
#define ADXL345_FULL_RES        (1 << 3)
#define ADXL345_JUSTIFY         (1 << 2)
#define ADXL345_RANGE(x)        ((x) & 0x3)

/* ADXL345_RANGE(x) options */
#define ADXL345_RANGE_PM_2G     0
#define ADXL345_RANGE_PM_4G     1
#define ADXL345_RANGE_PM_8G     2
#define ADXL345_RANGE_PM_16G    3

/* ADXL345_FIFO_CTL definition */
#define ADXL345_FIFO_MODE(x)    (((x) & 0x3) << 6)
#define ADXL345_TRIGGER         (1 << 5)
#define ADXL345_SAMPLES(x)      ((x) & 0x1F)

/* ADXL345_FIFO_MODE(x) options */
#define ADXL345_FIFO_BYPASS     0
#define ADXL345_FIFO_FIFO       1
#define ADXL345_FIFO_STREAM     2
#define ADXL345_FIFO_TRIGGER    3

/* ADXL345_FIFO_STATUS definition */
#define ADXL345_FIFO_TRIG       (1 << 7)
#define ADXL345_ENTRIES(x)      ((x) & 0x3F)

/* ADXL345 ID */
#define ADXL345_ID              0xE5

/* ADXL345 Full Resolution Scale Factor */
#define ADXL345_SCALE_FACTOR    0.0039

static adxl345_device_s *gp_accel_dev = NULL;
/* 写寄存器 */
static rt_err_t write_regs(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf) {
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = ADXL345_ADDRESS;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = ADXL345_ADDRESS;
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msgs[1].buf = buf;
    msgs[1].len = len;

    if (rt_i2c_transfer(bus, msgs, 2) == 2){
        return RT_EOK;
    }else{
        rt_kprintf("Writing command error.");
        return -RT_ERROR;
    }
}

/* 读寄存器 */
static rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf) {
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = ADXL345_ADDRESS;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = ADXL345_ADDRESS;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    if (rt_i2c_transfer(bus, msgs, 2) == 2){
        return RT_EOK;
    }else{
        //LOG_E("Reading command error.");
        return -RT_ERROR;
    }
}

rt_err_t sensor_accel_init(void){
    const char *i2c_bus_name = "i2c1";
    gp_accel_dev = adxl345_init(i2c_bus_name);
    if (gp_accel_dev == NULL) {
        rt_kprintf("The sensor adxl345 initializes failed!");
        return -RT_ERROR;
    }
    return RT_EOK;
}

rt_err_t sensor_accel_read(int16_t *x, int16_t *y, int16_t *z){
    rt_err_t result = -RT_ERROR;

    if (gp_accel_dev == NULL || x == NULL || y == NULL || z == NULL) {
        return -RT_ERROR;
    }
    result = adxl345_read(gp_accel_dev, x, y, z);
    return result;
}

/* 检测产品ID */
static rt_err_t check_device_ids(adxl345_device_s *pdev) {
    rt_err_t result = RT_EOK;
    rt_uint8_t did = 0;

    RT_ASSERT(pdev);

    /* 读取设备ID */
    result = read_regs(pdev->i2c, ADXL345_DEVID, 1, &did);
    if(result != RT_EOK){
        return -RT_ERROR;
    }
    if(did != ADXL345_ID){
        return -RT_ERROR;
    }
    return RT_EOK;
}


/* 传感器初始配置 */
static rt_err_t adxl345_init_config(adxl345_device_s *pdev){
    rt_err_t result = -RT_ERROR;
    rt_uint8_t reg_val = 0;

    RT_ASSERT(pdev);

    uint16_t mg = 0;
    mg /= 62.5; // 62.5mg/LSB
    if(mg > 255){
        rt_kprintf("adxl345 thresh act(%d) is more than max 255, set to 255", mg);
        mg = 255;
    }
    reg_val = (uint8_t)mg;
    result = write_regs(pdev->i2c, ADXL345_THRESH_ACT, 1, &reg_val);

    reg_val = (uint8_t)(mg / 2);
    result = write_regs(pdev->i2c, ADXL345_THRESH_INACT, 1, &reg_val);

    reg_val = 10; // 1s/LSB
    result = write_regs(pdev->i2c, ADXL345_TIME_INACT, 1, &reg_val);

    reg_val = 0;

    result = write_regs(pdev->i2c, ADXL345_ACT_INACT_CTL, 1, &reg_val);

    reg_val = ADXL345_RATE(0x08); // 0x08: 25Hz
    result = write_regs(pdev->i2c, ADXL345_BW_RATE, 1, &reg_val);

    reg_val = ADXL345_PCTL_LINK | ADXL345_PCTL_AUTO_SLEEP | ADXL345_PCTL_MEASURE | ADXL345_PCTL_WAKEUP(0); //链接使能, 静止自动休眠, 测量模式, 休眠时8Hz
    result = write_regs(pdev->i2c, ADXL345_POWER_CTL, 1, &reg_val);

    reg_val = ADXL345_ACTIVITY | ADXL345_INACTIVITY;
    result = write_regs(pdev->i2c, ADXL345_INT_ENABLE, 1, &reg_val);

    reg_val = 0; // 全部映射到INT1
    result = write_regs(pdev->i2c, ADXL345_INT_MAP, 1, &reg_val);

    reg_val = ADXL345_INT_INVERT | ADXL345_FULL_RES | ADXL345_RANGE(3);
    result = write_regs(pdev->i2c, ADXL345_DATA_FORMAT, 1, &reg_val);

    return result;
}

/* 传感器初始化 */
static rt_err_t adxl345_sensor_init(adxl345_device_s *pdev) {
    rt_err_t result = -RT_ERROR;

    RT_ASSERT(pdev);
    result = rt_mutex_take(pdev->lock, RT_WAITING_FOREVER);
    if (result != RT_EOK){
        goto __exit;
    }

    result = check_device_ids(pdev);
    if (result != RT_EOK){
        goto __exit;
    }

    result = adxl345_init_config(pdev);
    if(result != RT_EOK){
        goto __exit;
    }

__exit:
    if (result != RT_EOK){
        //LOG_E("This sensor initializes failure.");
    }
    rt_mutex_release(pdev->lock);

    return result;
}

/* 设备初始化 */
adxl345_device_s *adxl345_init(const char *i2c_bus_name){
    adxl345_device_s *pdev;

    RT_ASSERT(i2c_bus_name);
    pdev = rt_calloc(1, sizeof(adxl345_device_s));
    if (pdev == RT_NULL){
        rt_kprintf("Can't allocate memory for adxl345 device on '%s' ", i2c_bus_name);
        rt_free(pdev);
        return RT_NULL;
    }

    pdev->i2c = rt_i2c_bus_device_find(i2c_bus_name);
    if (pdev->i2c == RT_NULL) {
        rt_kprintf("Can't find adxl345 device on '%s'", i2c_bus_name);
        rt_free(pdev);
        return RT_NULL;
    }

    pdev->lock = rt_mutex_create("mutex_adxl345", RT_IPC_FLAG_FIFO);
    if (pdev->lock == RT_NULL){
        rt_kprintf("Can't create mutex for adxl345 device on '%s'", i2c_bus_name);
        rt_free(pdev);
        return RT_NULL;
    }

    /* init adxl345 sensor */
    if (adxl345_sensor_init(pdev) != RT_EOK){
        rt_kprintf("Can't init adxl345 device on '%s'", i2c_bus_name);
        rt_free(pdev);
        rt_mutex_delete(pdev->lock);
        return RT_NULL;
    }
    return pdev;
}

/* 读取当前加速度原始值 */
rt_err_t adxl345_read(adxl345_device_s *pdev, rt_int16_t *x, rt_int16_t *y, rt_int16_t *z){
    rt_err_t result = RT_ERROR;
    rt_uint8_t first_reg_address = ADXL345_DATAX0;
    rt_uint8_t read_buffer[7]    = {0};

    result = read_regs(pdev->i2c, first_reg_address, 6, read_buffer);
    if(result != RT_EOK){
        return -RT_ERROR;
    }
    *x = ((int16_t)read_buffer[1] << 8) + read_buffer[0];
    *y = ((int16_t)read_buffer[3] << 8) + read_buffer[2];
    *z = ((int16_t)read_buffer[5] << 8) + read_buffer[4];
    return result;
}

rt_err_t task_acc_start(uint8_t priority)
{
    rt_thread_t tid = RT_NULL;

    tid = rt_thread_create("acc",
                            acc_task_entry, RT_NULL,
                            2048,priority, 10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        return RT_EOK;
    }
    return -RT_ERROR;
}

void acc_task_entry(void *parameter)
{
    int16_t x,y,z;
    while (1)
    {
        sensor_accel_read(&x,&y,&z);
        rt_kprintf("acc:x=%d,y=%d,z=%d\n",x,y,z);
        rt_thread_mdelay(20);
    }
}
