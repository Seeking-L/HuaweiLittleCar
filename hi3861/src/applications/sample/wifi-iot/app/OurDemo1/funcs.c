#include <stdio.h>
#include <stdlib.h>

#include <hi_pwm.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_watchdog.h"
#include "iot_gpio.h"
#include "hi_time.h"
#include "hi_io.h"
#include "iot_gpio_ex.h"
#include <hi_gpio.h>
#include "iot_gpio_ex.h"

#define DELAY_US20 20
#define DELAY_MS10 10
#define GPIO_HIGH (1)
#define GPIO_LOW (0)

#define IOT_DUTY 99 // 占空比，控制电机的输出力度，越高小车越快
#define PWM_FREQ_FREQUENCY (65535)

// 探测时左边远还是右边远
#define LEFT 0
#define RIGHT 1

volatile int buttonFlag = 0;

void Hcsr04Init(void);
float GetDistance(void);
void OnButtonPressed(void);
void button_init(void);
void pwm_init(void);
void sg_init(void);
void pwm_stop();
void car_stop(void);
void pwm_forward(void);
void car_forward(void);
void pwm_backward(void);
void car_backward(void);
void pwm_left(void);
void car_left(void);
void pwm_right(void);
void car_right(void);
int detect();
hi_void engine_regress_middle(hi_void);
hi_void engine_turn_right(hi_void);
hi_void engine_turn_left(hi_void);
hi_void set_angle(hi_s32 duty);

void Hcsr04Init(void) // 超声波模块初始化
{
    /*
     * 设置超声波Echo为输入模式
     * 设置GPIO8功能（设置为GPIO功能）
     * Set ultrasonic echo as input mode
     * Set GPIO8 function (set as GPIO function)
     */
    IoSetFunc(IOT_IO_NAME_GPIO_8, IOT_IO_FUNC_GPIO_8_GPIO);
    /*
     * 设置GPIO8为输入方向
     * Set GPIO8 as the input direction
     */
    IoTGpioSetDir(IOT_IO_NAME_GPIO_8, IOT_GPIO_DIR_IN);
    /*
     * 设置GPIO7功能（设置为GPIO功能）
     * Set GPIO7 function (set as GPIO function)
     */
    IoSetFunc(IOT_IO_NAME_GPIO_7, IOT_IO_FUNC_GPIO_7_GPIO);
    /*
     * 设置GPIO7为输出方向
     * Set GPIO7 as the output direction
     */
    IoTGpioSetDir(IOT_IO_NAME_GPIO_7, IOT_GPIO_DIR_OUT);
}

float GetDistance(void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    /*
     * 设置GPIO7输出低电平
     * 给trig发送至少10us的高电平脉冲，以触发传感器测距
     * Set GPIO7 to output direction
     * Send a high level pulse of at least 10us to the trig to trigger the range measurement of the sensor
     */
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE1);
    hi_udelay(DELAY_US20);
    IoTGpioSetOutputVal(IOT_IO_NAME_GPIO_7, IOT_GPIO_VALUE0);
    /*
     * 计算与障碍物之间的距离
     * Calculate the distance from the obstacle
     */
    while (1)
    {
        /*
         * 获取GPIO8的输入电平状态
         * Get the input level status of GPIO8
         */
        IoTGpioGetInputVal(IOT_IO_NAME_GPIO_8, &value);
        /*
         * 判断GPIO8的输入电平是否为高电平并且flag为0
         * Judge whether the input level of GPIO8 is high and the flag is 0
         */
        if (value == IOT_GPIO_VALUE1 && flag == 0)
        {
            /*
             * 获取系统时间
             * get SysTime
             */
            start_time = hi_get_us();
            flag = 1;
        }
        /*
         * 判断GPIO8的输入电平是否为低电平并且flag为1
         * Judge whether the input level of GPIO8 is low and the flag is 1
         */
        if (value == IOT_GPIO_VALUE0 && flag == 1)
        {
            /*
             * 获取高电平持续时间
             * Get high level duration
             */
            time = hi_get_us() - start_time;
            break;
        }
    }
    /* 计算距离障碍物距离（340米/秒 转换为 0.034厘米/微秒, 2代表去来，两倍距离） */
    /* Calculate the distance from the obstacle */
    /* (340 m/s is converted to 0.034 cm/microsecond 2 represents going and coming, twice the distance) */
    distance = time * 0.034 / 2;
    printf("distance is %0.2f cm\r\n", distance);
    return distance;
}

void OnButtonPressed(void) // 按下按键，程序停止。
{
    // osDelay(20);
    // car_stop();
    // osDelay(20);
    osDelay(50);
    printf("Prograss Exit!");
    buttonFlag = 1;
}

void button_init(void) // 按钮模块初始化
{
    IoTGpioInit(IOT_IO_NAME_GPIO_5); // 5号是按钮，这里的作用是：按一次就程序停止
    hi_io_set_func(IOT_IO_NAME_GPIO_5, GPIO_LOW);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_5, IOT_GPIO_DIR_IN);
    hi_io_set_pull(IOT_IO_NAME_GPIO_5, GPIO_HIGH);
    IoTGpioRegisterIsrFunc(IOT_IO_NAME_GPIO_5, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_EDGE_FALL_LEVEL_LOW,
                           OnButtonPressed, NULL); // 使得按键可以打断程序的运行
}

void pwm_init(void)
{
    IoTGpioInit(IO_NAME_GPIO_0);
    IoTGpioInit(IO_NAME_GPIO_1);
    IoTGpioInit(IO_NAME_GPIO_9);
    IoTGpioInit(IO_NAME_GPIO_10);

    // 设置GPIO引脚复用
    hi_io_set_func(IO_NAME_GPIO_0, IO_FUNC_GPIO_0_PWM_OUT);
    hi_io_set_func(IO_NAME_GPIO_1, IO_FUNC_GPIO_1_PWM_OUT);
    hi_io_set_func(IO_NAME_GPIO_9, IO_FUNC_GPIO_9_PWM_OUT);
    hi_io_set_func(IO_NAME_GPIO_10, IO_FUNC_GPIO_10_PWM_OUT);

    // 设置输出方向
    hi_gpio_set_dir(IO_NAME_GPIO_0, HI_GPIO_DIR_OUT);
    hi_gpio_set_dir(IO_NAME_GPIO_1, HI_GPIO_DIR_OUT);
    hi_gpio_set_dir(IO_NAME_GPIO_9, HI_GPIO_DIR_OUT);
    hi_gpio_set_dir(IO_NAME_GPIO_10, HI_GPIO_DIR_OUT);

    // 初始化pwm
    IoTPwmInit(GPIO_0_PWM_PORT_PWM);
    IoTPwmInit(GPIO_1_PWM_PORT_PWM);
    IoTPwmInit(GPIO_9_PWM_PORT_PWM);
    IoTPwmInit(GPIO_10_PWM_PORT_PWM);
}

void pwm_stop()
{
    IoTPwmStop(GPIO_0_PWM_PORT_PWM);
    IoTPwmStop(GPIO_1_PWM_PORT_PWM);
    IoTPwmStop(GPIO_9_PWM_PORT_PWM);
    IoTPwmStop(GPIO_10_PWM_PORT_PWM);
}

void car_stop(void)
{
    printf("CAR STOP!\n");
    pwm_stop();
}

void pwm_forward(void)
{
    // 先停止
    car_stop();

    // 设置前进模式pwm
    IoTPwmStart(GPIO_0_PWM_PORT_PWM, (IOT_DUTY - 10), PWM_FREQ_FREQUENCY); // 左边马力大，所以要占空比小一点
    IoTPwmStart(GPIO_9_PWM_PORT_PWM, IOT_DUTY, PWM_FREQ_FREQUENCY);
}

void car_forward(void)
{
    printf("CAR FORWARD!\n");
    pwm_forward();
}

void pwm_backward(void)
{
    // 先停止
    car_stop();
    // 设置后退模式pwm
    IoTPwmStart(GPIO_1_PWM_PORT_PWM, IOT_DUTY, PWM_FREQ_FREQUENCY);
    IoTPwmStart(GPIO_10_PWM_PORT_PWM, IOT_DUTY, PWM_FREQ_FREQUENCY);
}

void car_backward(void)
{
    printf("CAR BACKWARD!\n");
    pwm_backward();
}

// 这个是原地左转
void pwm_left(void)
{
    // 先停止
    car_stop();
    // 设置单个电机pwm
    IoTPwmStart(GPIO_9_PWM_PORT_PWM, IOT_DUTY, PWM_FREQ_FREQUENCY);
}

void car_left(void)
{
    // 先停止
    car_stop();
    printf("CAR LEFT!\n");
    pwm_left();
}

// 这个是原地右转
void pwm_right(void)
{
    // 先停止
    car_stop();
    // 设置单个电机pwm
    IoTPwmStart(GPIO_0_PWM_PORT_PWM, IOT_DUTY, PWM_FREQ_FREQUENCY);
}

void car_right(void)
{
    printf("CAR RIGHT!\n");
    pwm_right();
}

// 下面是舵机的一套函数
hi_void set_angle(hi_s32 duty)
{
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_2, GPIO_HIGH);
    hi_udelay(duty);
    IoTGpioSetOutputVal(HI_IO_NAME_GPIO_2, GPIO_LOW);
    hi_udelay(20000 - duty);
}

// 舵机左转
hi_void engine_turn_left(hi_void)
{
    for (int i = 0; i < 10; i++)
    {
        set_angle(2500);
    }
}

// 舵机右转
hi_void engine_turn_right(hi_void)
{
    for (int i = 0; i < 10; i++)
    {
        set_angle(500);
    }
}

// 舵机归位
hi_void engine_regress_middle(hi_void)
{
    for (int i = 0; i < 10; i++)
    {
        set_angle(1500);
    }
}

// 电机模块初始化
void sg_init(void)
{
    hi_io_set_func(HI_IO_NAME_GPIO_2, 0);
    hi_gpio_set_dir(HI_IO_NAME_GPIO_2, HI_GPIO_DIR_OUT);
}

// 探测左右边的距离，返回的int类型为：0--左边障碍远  1--右边障碍远
int detect()
{
    osDelay(10);
    float left = 0.0, right = 0.0;
    engine_turn_left();

    osDelay(20); // TODO:这里有个疑惑：舵机左转的函数运行结束时，舵机是否已经左转到位了呢？若没有到位会导致测得的距离不对。右转同理。
    for (int i = 0; i < 10; i++)
    {
        left += GetDistance();
    }
    left /= 10;

    engine_regress_middle();
    engine_turn_right();
    osDelay(20);
    for (int i = 0; i < 10; i++)
    {
        right += GetDistance();
    }
    right /= 10;
    engine_regress_middle();
    return left > right ? LEFT : RIGHT;
}