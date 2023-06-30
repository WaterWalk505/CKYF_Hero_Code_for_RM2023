#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H
#include "mytype.h"


typedef struct
{
	/*实时变量*/
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t give_current;
	uint8_t temperature;
	int16_t last_ecd;
	int64_t total_ecd;
	/*目标变量*/
	uint16_t target_ecd;
	int64_t  target_total_ecd;
	int16_t  target_speed_rpm;
	int16_t  target_current;
	/*辅助计算累计编码器变量*/
	int32_t round_cnt;
	int16_t offset_ecd;
	uint8_t offset_flag;
}motor_data_t;

void calculate_motor3508_total_angle(motor_data_t *motor_data);

void send_motor_3508current_through_CAN1(int16_t motor1_current,int16_t motor2_current,int16_t motor3_current,int16_t motor4_current);
void get_motor3508_data(motor_data_t *motor_data,uint8_t *can_rx_data);
void get_motor6020_data(motor_data_t *motor_data,uint8_t *can_rx_data);

#endif
