#include "user_task.h"
#include "chassis.h"

extern "C" void Control_Task(void* argument)
{
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    // Gimbal_Init();
    // Balance_init();
    // 初始化只执行一次。
    Chassis_Init();

    for (;;)
    {
        // osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        uint32_t flags = osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);
        // Gimbal_Loop();
        // Balance_loop();
        if ((flags & osFlagsError) != 0U)
        {
            continue;
        }

        // 初始化部分失败后仍需周期处理未成功入队的 Disable；是否允许运动由底盘内部判定。
        Chassis_Loop();
    }
}
