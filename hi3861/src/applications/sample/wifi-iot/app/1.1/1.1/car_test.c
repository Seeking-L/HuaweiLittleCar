#include <stdio.h>

#include <unistd.h>

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
#include "car_test.h"
#include "udp_test.h"
#include "iot_pwm.h"

#define GPIO0 0
#define GPIO1 1
#define GPIO9 9
#define GPIO10 10
#define GPIOFUNC 0
#define PWM_FREQ_FREQUENCY  (60000)

void gpio_control (unsigned int  gpio, IotGpioValue value) {
    hi_io_set_func(gpio, GPIOFUNC);
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(gpio, value);
}

struct car_sys_info car_info;

//CarStatus carstatus = CAR_STATUS_STOP;
//CarMode carmode = CAR_MODE_STEP;

void car_info_init(void)
{
	car_info.go_status = CAR_STATUS_STOP;
	car_info.cur_status = CAR_STATUS_STOP;

	car_info.mode = CAR_MODE_STEP;

	car_info.step_count = CAR_STEP_COUNT;
}

void step_count_update(void)
{
	car_info.step_count = CAR_STEP_COUNT;
}

void set_car_status(CarStatus status)
{
	if(status != car_info.cur_status)
	{
		car_info.status_change = 1;
	}
	car_info.go_status = status;

	step_count_update();
}


//设置行驶模式
void set_car_mode(CarMode mode)
{
	car_info.mode = mode;
}

//停止
void p_stop(void)
{
	//先停止PWM
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE1);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE1);
}
void c_stop(void)
{
	car_info.cur_status = car_info.go_status;

	printf("p_stop \r\n");

	p_stop();
}

//前进
void p_forward(void)
{
	//先停止PWM
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);

	//启动A路PWM
	//����
	//IoTPwmStart(p_PORT_PWM3, 64000, 64000);
	//����
	//IoTPwmStart(p_PORT_PWM0, 64000, 64000);
}
void c_forward(void)
{
	if(car_info.go_status != CAR_STATUS_FORWARD)
	{
		//直接退�?
		return ;
	}
	if(car_info.cur_status == car_info.go_status)
	{
		//状态没有变化，直接推出
		return;
	}

	car_info.cur_status = car_info.go_status;

	printf("p_forward \r\n");

	p_forward();

	step_count_update();
}

//后退
void p_backward(void)
{
	//先停止PWM
    gpio_control(GPIO0, IOT_GPIO_VALUE0);
    gpio_control(GPIO1, IOT_GPIO_VALUE1);
    gpio_control(GPIO9, IOT_GPIO_VALUE0);
    gpio_control(GPIO10, IOT_GPIO_VALUE1);

	//启动A路PWM
	//IoTPwmStart(p_PORT_PWM4, 64000, 64000);
	//IoTPwmStart(p_PORT_PWM1, 64000, 64000);
}
void c_backward(void)
{
	if(car_info.go_status != CAR_STATUS_BACKWARD)
	{
		//直接退�?
		return ;
	}
	if(car_info.cur_status == car_info.go_status)
	{
		//状态没有变化，直接推出
		return;
	}

	car_info.cur_status = car_info.go_status;

	printf("p_backward \r\n");

	p_backward();

	step_count_update();
}

//左转
void p_left(void)
{
	//先停止PWM
    gpio_control(GPIO0, IOT_GPIO_VALUE0);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE1);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);

	//启动A路PWM
	//IoTPwmStart(p_PORT_PWM0, 64000, 64000);

	
}
void c_left(void)
{
	if(car_info.go_status != CAR_STATUS_LEFT)
	{
		//直接退�?
		return ;
	}
	if(car_info.cur_status == car_info.go_status)
	{
		//状态没有变化，直接推出
		return;
	}

	car_info.cur_status = car_info.go_status;

	printf("p_left \r\n");

	p_left();

	step_count_update();
}

//右转
void p_right(void)
{
	//先停止PWM
    gpio_control(GPIO0, IOT_GPIO_VALUE1);
    gpio_control(GPIO1, IOT_GPIO_VALUE0);
    gpio_control(GPIO9, IOT_GPIO_VALUE0);
    gpio_control(GPIO10, IOT_GPIO_VALUE0);

	//启动A路PWM
	//IoTPwmStart(p_PORT_PWM3, 64000, 64000);
}
void c_right(void)
{
	if(car_info.go_status != CAR_STATUS_RIGHT)
	{
		//直接退�?
		return ;
	}
	if(car_info.cur_status == car_info.go_status)
	{
		//状态没有变化，直接推出
		return;
	}

	car_info.cur_status = car_info.go_status;

	printf("p_right \r\n");

	p_right();

	step_count_update();
}

extern void start_udp_thread(void);

void car_test(void)
{
	start_udp_thread();
	car_info_init();

	while(1)
	{
		if(car_info.status_change)
		{
			car_info.status_change = 0;

			switch(car_info.go_status)
			{
				case CHANGE_MODE:
					c_stop();
					break;
				
				case CAR_STATUS_FORWARD:
					c_forward();
					break;

				case CAR_STATUS_BACKWARD:
					c_backward();
					break;

				case CAR_STATUS_LEFT:
					c_left();
					break;

				case CAR_STATUS_RIGHT:
					c_right();
					break;

				default:

					break;
			}
		}
		

		if(car_info.mode == CAR_MODE_STEP)
		{
			if(car_info.go_status != CAR_STATUS_STOP)
			{
				if(car_info.step_count > 0)
				{
					car_info.step_count --;
				}else{
					printf("stop... \r\n");
					set_car_status(CAR_STATUS_STOP);
				}
			}
		}

		usleep(1000);
	}
	
}

