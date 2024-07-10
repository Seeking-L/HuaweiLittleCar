/*控制电机的部分*/

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

#include "motor.h"

// 初始化PWM
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

// 停止pwm输出
void pwm_stop()
{
    IoTPwmStop(GPIO_0_PWM_PORT_PWM);
    IoTPwmStop(GPIO_1_PWM_PORT_PWM);
    IoTPwmStop(GPIO_9_PWM_PORT_PWM);
    IoTPwmStop(GPIO_10_PWM_PORT_PWM);
}
// 停车
void car_stop(void)
{
    pwm_stop();
}
// 小车前进的pwm模式
void pwm_forward(void)
{
    // 先停止
    car_stop();

    // 设置前进模式pwm
    IoTPwmStart(GPIO_0_PWM_PORT_PWM, IOT_DUTY_L, PWM_FREQ_FREQUENCY);
    IoTPwmStart(GPIO_9_PWM_PORT_PWM, IOT_DUTY_R, PWM_FREQ_FREQUENCY);
}
// 小车前进
void car_forward(void)
{
    pwm_forward();
}
// 小车后退的pwm模式
void pwm_backward(void)
{
    // 先停止
    car_stop();
    // 设置后退模式pwm
    IoTPwmStart(GPIO_1_PWM_PORT_PWM, IOT_DUTY_L, PWM_FREQ_FREQUENCY);
    IoTPwmStart(GPIO_10_PWM_PORT_PWM, IOT_DUTY_R, PWM_FREQ_FREQUENCY);
}
// 小车后退
void car_backward(void)
{
    pwm_backward();
}
// 这个是原地左转的pwm模式
void pwm_left(void)
{
    // 先停止
    car_stop();
    // 设置单个电机pwm
    IoTPwmStart(GPIO_9_PWM_PORT_PWM, IOT_DUTY_R, PWM_FREQ_FREQUENCY);
    IoTPwmStart(GPIO_1_PWM_PORT_PWM, IOT_DUTY_L, PWM_FREQ_FREQUENCY);
}
// 原地左转
void car_left(void)
{
    // 先停止
    car_stop();
    pwm_left();
}
// 这个是原地右转的pwm模式
void pwm_right(void)
{
    // 先停止
    car_stop();
    // 设置单个电机pwm
    IoTPwmStart(GPIO_0_PWM_PORT_PWM, IOT_DUTY_L, PWM_FREQ_FREQUENCY);
    IoTPwmStart(GPIO_10_PWM_PORT_PWM, IOT_DUTY_R, PWM_FREQ_FREQUENCY);
}
// 原地右转
void car_right(void)
{
    // 先停止
    car_stop();
    pwm_right();
}
/*测试这个motor.c用的函数
 void car_test(void)
 {
     pwm_init();
     //前进
     car_forward()
     //右转
     car_right()
     //左转
     car_left()
     //后退
     car_backward();
 }
 */
