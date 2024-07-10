#include <stdio.h>

#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
// #include "iot_gpio.h"
#include "hi_types.h"
#include "hi_time.h"

#define DELAY_TICKS_30 (30) // 延迟30ms

// 配置“线程属性”用的。还没仔细看
#define LJQ_TASK_STACK_SIZE 512 // 还没仔细看，可能是规定内存之类的？
#define LJQ_TASK_PRIO (25)

// 舵机的2号GPIO口
#define HI_IO_NAME_GPIO_2 2

// 高低电平？
#define GPIO_HIGH (1)
#define GPIO_LOW (0)

typedef enum
{
    HI_GPIO_DIR_IN = 0, /**< Input.CNcomment:输入方向 CNend*/
    HI_GPIO_DIR_OUT     /**< Output.CNcomment:输出方向 CNend*/
} hi_gpio_dir;

enum SGState // Steering gear舵机
{
    SG_LEFT = 0,
    SG_MIDDLE,
    SG_RIGHT,
};

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

// 初始参数
static int g_count = 11;            // 运动次数
enum SGState g_SGState = SG_MIDDLE; // 初始状态

static void *LjqTest1(void)
{
    while (g_count)
    {
        switch (g_SGState)
        {
        case SG_LEFT:
            printf("now is left---g_count=%d\n", g_count); // 调试用
            g_SGState = SG_MIDDLE;
            engine_regress_middle();
            osDelay(DELAY_TICKS_30);
            break;
        case SG_MIDDLE:
            printf("now is middle---g_count=%d\n", g_count); // 调试用
            if (g_count % 3 == 1)
            {
                g_SGState = SG_LEFT;
                engine_turn_left();
            }
            else
            {
                g_SGState = SG_RIGHT;
                engine_turn_right();
                osDelay(DELAY_TICKS_30);
            }
            break;
        case SG_RIGHT:
            printf("now is right---g_count=%d\n", g_count); // 调试用
            g_SGState = SG_MIDDLE;
            engine_regress_middle();
            osDelay(DELAY_TICKS_30);
            break;
        default:
            osDelay(DELAY_TICKS_30);
            break;
        }

        g_count--;
    }
    engine_regress_middle(); // 最后回位
    return NULL;
}

static void LjqTest1Entry(void)
{
    osThreadAttr_t attr;

    hi_io_set_func(HI_IO_NAME_GPIO_2, 0);
    hi_gpio_set_dir(HI_IO_NAME_GPIO_2, HI_GPIO_DIR_OUT);

    // 我也没仔细搞清楚有啥用，但是demo都加了，抄上去就完事了（吧）
    attr.name = "LjqTest1";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = LJQ_TASK_STACK_SIZE;
    attr.priority = LJQ_TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)LjqTest1, NULL, &attr) == NULL)
    {
        printf("[LjqTest1] Falied to create LjqTest1!\n");
    }
}
SYS_RUN(LjqTest1Entry);