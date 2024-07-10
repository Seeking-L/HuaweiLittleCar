#ifndef __BAI_CAR_H__
#define __BAI_CAR_H__

/*这一块分别是GPIO 0, GPIO 1, GPIO 9, GPIO 10所对应GPIO id，用数字表示*/
#define IO_NAME_GPIO_0 0
#define IO_NAME_GPIO_1 1
#define IO_NAME_GPIO_9 9
#define IO_NAME_GPIO_10 10

/*这一块分别是GPIO 0, GPIO 1, GPIO 9, GPIO 10所对应的pwm模式*/
#define IO_FUNC_GPIO_0_PWM_OUT HI_IO_FUNC_GPIO_0_PWM3_OUT
#define IO_FUNC_GPIO_1_PWM_OUT HI_IO_FUNC_GPIO_1_PWM4_OUT
#define IO_FUNC_GPIO_9_PWM_OUT HI_IO_FUNC_GPIO_9_PWM0_OUT
#define IO_FUNC_GPIO_10_PWM_OUT HI_IO_FUNC_GPIO_10_PWM1_OUT

/*这一块分别是GPIO 0, GPIO 1, GPIO 9, GPIO 10所对应的pwm端口*/
#define GPIO_0_PWM_PORT_PWM HI_PWM_PORT_PWM3
#define GPIO_1_PWM_PORT_PWM HI_PWM_PORT_PWM4
#define GPIO_9_PWM_PORT_PWM HI_PWM_PORT_PWM0
#define GPIO_10_PWM_PORT_PWM HI_PWM_PORT_PWM1

// 控制左轮U1前进口
#define GPIO0 0
// 控制左轮U1后退口
#define GPIO1 1
// 控制右轮U2前进口
#define GPIO9 9
// 控制右轮U2后退口
#define GPIO10 10
// pwm占空比
#define IOT_DUTY_L 99 
#define IOT_DUTY_R 85
// 小车pwm频率分频系数
#define PWM_FREQ_FREQUENCY 65535

void car_stop(void);
void car_forward(void);
void car_backward(void);
void car_left(void);
void car_right(void);
void pwm_init(void);

#endif
