#include "motor.h"
#include "ultrasonic.h"
#include "servo.h"

// #include <math.h>
// #include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "hi_timer.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio_ex.h"
#include "iot_gpio.h"
#include "iot_pwm.h"
#include "iot_errno.h"
#include "hi_errno.h"
#include "hi_io.h"
#include "iot_watchdog.h"
#include "hi_time.h"

// 注意记得加上math库,and bool 的路径!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/*这里假设小车只能沿着x轴和y轴方向移动，不能沿着其他方向移动，
  以方便计算速度与当前位置坐标
*/

// 这是目的地的坐标位置，默认Y为0
#define X0 12.0

// 这是判断距离目的地0.5m的时候就认为已经到达目的地了
#define DISTANCE_LIMIT 1.0

// 这是速度的大小，以米为单位
#define SPEED_VALUE 0.8

// 这是距离障碍多少会发出警报的最大距离，以cm为单位
#define WARNING_LIMIT 20

/*这个时间是小车在没有障碍行驶的时候所能
  行驶的最大时间，单位是ms，超过这时间就要往目的地走*/
#define CONTINUE_TIME 2000

/*控制pwm延长时间，也就是转向的角度大小*/
#define PWM_DELAY_TIME 40

// 既然可以原地转圈，那就不同回退了
// /*控制小车后退延长时间，也就是后退了多少*/
// #define BACK_DELAY_TIME 60

#define DIR 0
#define PTR 1

// 这里是显示小车的速度方向
typedef enum
{
    DIR_X,
    DIR_Y
} DIRECTION;

// 这里是显示小车的速度方向正负
typedef enum
{
    NEG,
    POS
} NEG_OR_POS;

/*..._speed_state[0]记录速度方向是X轴或Y轴
  ..._speed_state[1]记录正半轴或者负半轴*/
volatile int expect_speed_state[2] = {0, 0};
volatile int current_speed_state[2] = {0, 0};

// 这是距目的地距离,以米为单位
volatile float distance_to_destination = X0;

volatile float start_position[2] = {0.0, 0.0};

volatile float destination_position[2] = {X0, 0.0}; // 这里假设以目的地和起始地的连线为x轴

volatile float current_position[2] = {0.0, 0.0};

volatile static int buttonState = 0; // 按键被按下的标志

// 取根号函数
float InvSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int *)&x;             // get bits for floating VALUE
    i = 0x5f375a86 - (i >> 1);      // gives initial guess y0
    x = *(float *)&i;               // convert bits BACK to float
    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy

    return 1 / x;
}

void LeftKeyPressed(void)
{
    buttonState = 1;
}

void RightKeyPressed(void)
{
    buttonState = 0;
}

void KeyInit(void)
{
    /*
     *这是左边按键，就是S1的，使小车中断
     * 使能GPIO5的中断功能, OnFuncKeyPressed 为中断的回调函数*/
    IoTGpioRegisterIsrFunc(IOT_IO_NAME_GPIO_5, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_EDGE_FALL_LEVEL_LOW, LeftKeyPressed, NULL);

    /*
     *这是右边按键，就是S2的，使小车恢复
     * 使能GPIO7的中断功能, OnFuncKeyPressed 为中断的回调函数*/
    IoTGpioRegisterIsrFunc(IOT_IO_NAME_GPIO_7, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_EDGE_FALL_LEVEL_LOW, RightKeyPressed, NULL);
}

void car_init(void)
{
    /*超声波模块初始化*/
    Hcsr04Init();
    /*舵机模块初始化*/
    S92RInit();
    /*电机模块初始化*/
    pwm_init();
    // 初始化小车速度
    speed_init();
    // /*按键初始化*/
    // KeyInit();
}

// 初始化小车速度
void speed_init()
{
    expect_speed_state[DIR] = DIR_X; // 初始方向是DIR_X，即目的地的方向
    expect_speed_state[PTR] = POS;   // 正方向
    current_speed_state[DIR] = DIR_X;
    current_speed_state[PTR] = POS;
}

void change_speed_state()
{
    car_stop();
    // 这些是判断车速度状态与期望状态差别，并往期望状态转变
    // 这些判断的前提就是车辆不会原路返回，即DIR相同时，PTR（方向）一定一样
    if (current_speed_state[DIR] == expect_speed_state[DIR])
    {
        // 更新车辆行驶速度状态
        current_speed_state[DIR] = expect_speed_state[DIR];
        current_speed_state[PTR] = expect_speed_state[PTR];
        return;
    }
    //!!! 是否需要判断x或y
    else if (current_speed_state[DIR] == DIR_Y &&
             current_speed_state[PTR] == POS &&
             expect_speed_state[PTR] == POS)
    {
        car_right();
        osDelay(PWM_DELAY_TIME);
    }
    else if (current_speed_state[DIR] == DIR_X &&
             current_speed_state[PTR] == POS &&
             expect_speed_state[PTR] == POS)
    {
        car_left();
        osDelay(PWM_DELAY_TIME);
    }

    else if (current_speed_state[DIR] == DIR_Y &&
             current_speed_state[PTR] == POS &&
             expect_speed_state[PTR] == NEG)
    {
        car_left();
        osDelay(PWM_DELAY_TIME);
    }
    else if (current_speed_state[DIR] == DIR_X &&
             current_speed_state[PTR] == POS &&
             expect_speed_state[PTR] == NEG)
    {
        car_right();
        osDelay(PWM_DELAY_TIME);
    }

    else if (current_speed_state[DIR] == DIR_Y &&
             current_speed_state[PTR] == NEG &&
             expect_speed_state[PTR] == POS)
    {
        car_left();
        osDelay(PWM_DELAY_TIME);
    }
    else if (current_speed_state[DIR] == DIR_X &&
             current_speed_state[PTR] == NEG &&
             expect_speed_state[PTR] == POS)
    {
        car_right();
        osDelay(PWM_DELAY_TIME);
    }

    else if (current_speed_state[DIR] == DIR_Y &&
             current_speed_state[PTR] == NEG &&
             expect_speed_state[PTR] == NEG)
    {
        car_right();
        osDelay(PWM_DELAY_TIME);
    }
    else if (current_speed_state[DIR] == DIR_X &&
             current_speed_state[PTR] == NEG &&
             expect_speed_state[PTR] == NEG)
    {
        car_left();
        osDelay(PWM_DELAY_TIME);
    }
    // // 更新车辆行驶速度状态
    // current_speed_state[DIR] = expect_speed_state[DIR];
    // current_speed_state[PTR] = expect_speed_state[PTR];
}

int car_move()
{
    car_stop();
    change_speed_state();
    car_forward();
    int start_time = hi_get_milli_seconds();
    return start_time;
}

// 在last_position和current_position之间小车速度大小和方向都不改变
// start_time是上一次停下之后再出发的时间
void get_current_position(float last_position[], int last_start_time)
{
    // 距离目的地距离X方向大小
    float distance_x = destination_position[DIR_X] - current_position[DIR_X];
    // 距离目的地距离Y方向大小
    float distance_y = destination_position[DIR_Y] - current_position[DIR_Y];
    // 总的距离大小
    distance_to_destination = InvSqrt((distance_x * distance_x + distance_y * distance_y));

    int current_time = hi_get_milli_seconds();
    int delta_time = current_time - last_start_time; // 运动的时间
    float delta_s = SPEED_VALUE * delta_time * 0.001;
    switch (current_speed_state[DIR])
    {
    case DIR_X:
        current_position[DIR_X] = last_position[DIR_X] + delta_s;
        break;

    case DIR_Y:
        current_position[DIR_Y] = last_position[DIR_Y] + delta_s;
        break;
    }
    last_position[DIR_X] = current_position[DIR_X];
    last_position[DIR_Y] = current_position[DIR_Y];
}

// 这个函数是调整小车往目的地方向去
// 返回开始运动时间
void find_destination()
{
    // 距离目的地距离X方向大小
    float distance_x = destination_position[DIR_X] - current_position[DIR_X];
    // 距离目的地距离Y方向大小
    float distance_y = destination_position[DIR_Y] - current_position[DIR_Y];
    // if(abs(distance_x)<=abs(distance_y))
    float abs_distance_x = distance_x;
    float abs_distance_y = distance_y;
    if (distance_x < 0)
        abs_distance_x = -1.0 * distance_x;
    if (distance_y < 0)
        abs_distance_y = -1.0 * distance_y;
    // 这是先把远距离走完了再走近距离地
    if (abs_distance_x >= abs_distance_y)
    {
    L_X:
        expect_speed_state[DIR] = DIR_X;
        if (distance_x < 0)
            expect_speed_state[PTR] = NEG;
        else
            expect_speed_state[PTR] = POS;
        if (current_speed_state[DIR] == expect_speed_state[DIR] && current_speed_state[PTR] != expect_speed_state[PTR]) // 这个是检测小车是不是原路返回了
        {
            goto L_Y;
        }
    }
    else
    {
    L_Y:
        expect_speed_state[DIR] = DIR_Y;
        if (distance_y < 0)
            expect_speed_state[PTR] = NEG;
        else
            expect_speed_state[PTR] = POS;
        if (current_speed_state[DIR] == expect_speed_state[DIR] && current_speed_state[PTR] != expect_speed_state[PTR]) // 这个是检测小车是不是原路返回了
        {
            goto L_X;
        }
    }
}

// 启动小车,返回启动时间
// 因为本身就朝向目的地，所以不用调用find-destination了
int car_start()
{
    int last_start_time;
    RegressMiddle();
    car_init();
    last_start_time = car_move();
    printf("Trip started!    ");
    printf("Distance to destination:%f\n", distance_to_destination);
    return last_start_time;
}

// 检测距离障碍，返回是否有障碍，1是有障碍，0是没有
int detect(void)
{
    float distance_to_barrier = GetDistance();
    if (distance_to_barrier < WARNING_LIMIT)
        return 1;
    else
        return 0;
}

void avoid_bar(float limit)
{
    // 0就意味着右转，1意味着左转
    int LEFT = 0;
    // 这个flag是判断小车行驶的方向：
    // 0x00是X轴负方向，0x01是X轴正方向，0x10是Y轴负方向，0x11是Y轴正方向
    int flag = 0x00;
    switch (current_speed_state[DIR])
    {
    case DIR_X:
        if (current_speed_state[PTR] == NEG)
            flag = 0x00;
        else
            flag = 0x01;
        break;
    case DIR_Y:
        if (current_speed_state[PTR] == NEG)
            flag = 0x10;
        else
            flag = 0x11;
    }
    // 舵机回中
    RegressMiddle();
    float left_distance;
    float right_distance;
    EngineTurnRight();
    osDelay(80);
    car_stop();
    right_distance = GetDistance(); //!!! getDistance()的作用？
    EngineTurnLeft();
    osDelay(80);
    car_stop();
    RegressMiddle();
    left_distance = GetDistance();
    LEFT = left_distance > right_distance ? 1 : 0;
    // expect_speed_state[DIR]是与当前运动方向垂直的方向
    expect_speed_state[DIR] = !current_speed_state[DIR];
    // 确定前进方向
    switch (flag)
    {
    case 0x00:
        if (LEFT)
            expect_speed_state[PTR] = NEG;
        else
            expect_speed_state[PTR] = POS;
        break;
    case 0x01:
        if (LEFT)
            expect_speed_state[PTR] = POS;
        else
            expect_speed_state[PTR] = NEG;
        break;
    case 0x10:
        if (LEFT)
            expect_speed_state[PTR] = POS;
        else
            expect_speed_state[PTR] = NEG;
        break;
    case 0X11:
        if (LEFT)
            expect_speed_state[PTR] = NEG;
        else
            expect_speed_state[PTR] = POS;
        break;
    }
    RegressMiddle();
}

void FinalTestTask()
{
    float last_position[2] = {0.0, 0.0};
    int current_time = 0;
    // 启动小车
    int last_start_time = car_start();
    last_position[DIR_X] = start_position[DIR_X];
    last_position[DIR_Y] = start_position[DIR_Y];
    while (distance_to_destination > DISTANCE_LIMIT)
    {
        current_time = hi_get_milli_seconds();
        if (detect() == 1)
        {
            car_stop();
            get_current_position(last_position, last_start_time);
            avoid_bar(WARNING_LIMIT);
            // 既然可以原地转圈，那就不同回退了
            //  car_backward();
            //  osDelay(BACK_DELAY_TIME);
            //  car_stop();
            last_start_time = car_move();
        }
        else if ((current_time - last_start_time) >= CONTINUE_TIME)
        {
            car_stop();
            get_current_position(last_position, last_start_time);
            find_destination();
            last_start_time = car_move();
        }
        printf("TIME:%d            , Distance to destination:%f\n", current_time, distance_to_destination);
    }
    car_stop();
    RegressMiddle();
    printf("\n******Arrived!******\n");
}

void FinalTestSampleEntry(void)
{
    osThreadAttr_t attr;

    attr.name = "FinalTestTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 5; // 堆栈大a小为1024*5 stack size 5*1024
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)FinalTestTask, NULL, &attr) == NULL)
    {
        printf("[FinalTestTask] Failed to create FinalTestTask!\n");
    }
}
SYS_RUN(FinalTestSampleEntry);