#include <stdio.h>

#include "ohos_init.h"
#include "cmsis_os2.h"

#include <iot_pwm.h>
#include <iot_gpio.h>

#include <unistd.h>
#include <hi_types_base.h>
#include <hi_early_debug.h>

#include <hi_pwm.h>
#include <hi_gpio.h>
#include <hi_io.h>

#include "iot_pwm.h"

#include "bai_car.h"

#define GPIO0 0
#define GPIO1 1
#define GPIO9 9
#define GPIO10 10
#define GPIO_STOP_FUNC 0
#define IOT_DUTY            70
#define PWM_FREQ_FREQUENCY  (65535)

void pwm_init(void)
{
    IoTGpioInit(IO_NAME_GPIO_0);
    IoTGpioInit(IO_NAME_GPIO_1);
    IoTGpioInit(IO_NAME_GPIO_9);
    IoTGpioInit(IO_NAME_GPIO_10);

    //设置GPIO引脚复用
    hi_io_set_func(IO_NAME_GPIO_0,IO_FUNC_GPIO_0_PWM_OUT);
    hi_io_set_func(IO_NAME_GPIO_1,IO_FUNC_GPIO_1_PWM_OUT);
    hi_io_set_func(IO_NAME_GPIO_9,IO_FUNC_GPIO_9_PWM_OUT);
    hi_io_set_func(IO_NAME_GPIO_10,IO_FUNC_GPIO_10_PWM_OUT);

    //设置输出方向
    hi_gpio_set_dir(IO_NAME_GPIO_0,HI_GPIO_DIR_OUT);
    hi_gpio_set_dir(IO_NAME_GPIO_1,HI_GPIO_DIR_OUT);
    hi_gpio_set_dir(IO_NAME_GPIO_9,HI_GPIO_DIR_OUT);
    hi_gpio_set_dir(IO_NAME_GPIO_10,HI_GPIO_DIR_OUT);

    //初始化pwm
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
    //先停止
    car_stop();

    //设置前进模式pwm
    IoTPwmStart(GPIO_0_PWM_PORT_PWM,IOT_DUTY,PWM_FREQ_FREQUENCY);
    IoTPwmStart(GPIO_9_PWM_PORT_PWM,IOT_DUTY,PWM_FREQ_FREQUENCY);
}

void car_forward(void)
{
    printf("CAR FORWARD!\n");
    pwm_forward();
}

void pwm_backward(void)
{
    //先停止
    car_stop();
    //设置后退模式pwm
    IoTPwmStart(GPIO_1_PWM_PORT_PWM,IOT_DUTY,PWM_FREQ_FREQUENCY);
    IoTPwmStart(GPIO_10_PWM_PORT_PWM,IOT_DUTY,PWM_FREQ_FREQUENCY);
}

void car_backward(void)
{
    printf("CAR BACKWARD!\n");
    pwm_backward();
}

//这个是原地左转
void pwm_left(void)
{
    //先停止
    car_stop();
    //设置单个电机pwm
    IoTPwmStart(GPIO_10_PWM_PORT_PWM,IOT_DUTY,PWM_FREQ_FREQUENCY);
}

void car_left(void)
{
    //先停止
    car_stop();
    printf("CAR LEFT!\n");
    pwm_left();
}

//这个是原地右转
void pwm_right(void)
{
    //先停止
    car_stop();
    //设置单个电机pwm
    IoTPwmStart(GPIO_0_PWM_PORT_PWM,IOT_DUTY,PWM_FREQ_FREQUENCY);
}

void car_right(void)
{
    printf("CAR RIGHT!\n");
    pwm_right();
}

void car_test(void)
{
    pwm_init();
    //前进
    car_forward();

    //右转
    car_right();

    //左转
    car_left();

    //后退
    car_backward();
}

void car_test_task(void)
{
    car_test();
}


void BAICarSampleEntry(void)
{
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "car_test_task";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 5; // 堆栈大小为1024*5,stack size 1024*5
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)car_test_task, NULL, &attr) == NULL) {
        printf("[car_test_task] Failed to create Hcsr04SampleTask!\n");
    }
}

APP_FEATURE_INIT(BAICarSampleEntry);