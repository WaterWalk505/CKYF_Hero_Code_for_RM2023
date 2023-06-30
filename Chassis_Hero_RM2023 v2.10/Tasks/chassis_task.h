#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "mytype.h"

/* 底盘功率限制宏定义 */
#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f
#define BUFFER_TOTAL_CURRENT_LIMIT   16000.0f
#define POWER_TOTAL_CURRENT_LIMIT    20000.0f
#define WARNING_POWER_BUFF           60.0f

void Chassis_PowerLimitControl(void);
void Chassis_PowerCapControl(void);

#endif
