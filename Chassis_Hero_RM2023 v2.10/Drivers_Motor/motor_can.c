#include "motor_can.h"
#include "can.h"
#include "chassis_motor.h"

/*****************************************[1]	发送协议*****************************************************************/

/**	[1.1]					 	
  * brief	  	     向电机发送目标电流值
  * param[in]      motor_current[0]:ID=1	底盘3508电机	电流
	*															[1]:ID=2	底盘3508电机	电流		
	*															[2]:ID=3	底盘3508电机	电流		
	*															[3]:ID=4	底盘3508电机	电流			
	* postscript		 通信：CAN1 	电机协议：ID从1到4的3508电机协议*/
int16_t motor_3508current_on_CAN1_setting[4];//统一赋值，统一发送
void send_motor_3508current_through_CAN1(int16_t motor1_current,int16_t motor2_current,int16_t motor3_current,int16_t motor4_current)
{
	CAN_TxHeaderTypeDef motor3508_current_setting_tx_message;
	uint8_t motor3508_current_setting_can_send_data[8];
	uint32_t send_mail_box;
	
	motor3508_current_setting_tx_message.StdId=0x200;//ID从1到4
	motor3508_current_setting_tx_message.RTR=CAN_RTR_DATA;
	motor3508_current_setting_tx_message.IDE=CAN_ID_STD;
	motor3508_current_setting_tx_message.DLC=0x08;
	
	motor3508_current_setting_can_send_data[0]=motor1_current >> 8;
	motor3508_current_setting_can_send_data[1]=motor1_current & 0xff;
	motor3508_current_setting_can_send_data[2]=motor2_current >> 8;
	motor3508_current_setting_can_send_data[3]=motor2_current & 0xff;
	motor3508_current_setting_can_send_data[4]=motor3_current >> 8;
	motor3508_current_setting_can_send_data[5]=motor3_current & 0xff;
	motor3508_current_setting_can_send_data[6]=motor4_current >> 8;
	motor3508_current_setting_can_send_data[7]=motor4_current & 0xff;
	
	HAL_CAN_AddTxMessage(&hcan1,&motor3508_current_setting_tx_message,motor3508_current_setting_can_send_data,&send_mail_box);
}




/*****************************************[2]	接收协议*****************************************************************/


/**	[2.1]					 获取电机反馈数据
  * brief          将电机电调通过can反馈的数据，按照3508报文格式，解析到motor_data_t型结构体中
  * param[in]      motor_data_t *motor_data：		包含电机位置、速度、电流等信息，解析后的电机数据放在这个结构体里面
  * param[in]      uint8_t *can_rx_data：				can接收到的电机反馈的原始数据
	* postscript		 在canRX回调函数里面使用																				*/
void get_motor3508_data(motor_data_t *motor_data,uint8_t *can_rx_data)
{
		motor_data->last_ecd 		= motor_data->ecd;
		motor_data->ecd      		= (uint16_t)(can_rx_data[0]<<8 |can_rx_data[1]);
		motor_data->speed_rpm		=	(uint16_t)(can_rx_data[2]<<8 |can_rx_data[3]);
		motor_data->give_current=	(uint16_t)(can_rx_data[4]<<8 |can_rx_data[5]);
		motor_data->temperature	=	can_rx_data[6];
}

/**	[2.2]					 获取电机反馈数据
  * brief          将电机电调通过can反馈的数据，按照6020报文格式，解析到motor_data_t型结构体中
  * param[in]      motor_data_t *motor_data：		包含电机位置、速度、电流等信息，解析后的电机数据放在这个结构体里面
  * param[in]      uint8_t *can_rx_data：				can接收到的电机反馈的原始数据
	* postscript		 在canRX回调函数里面使用																				*/
motor_data_t yaw_motor_data;
void get_motor6020_data(motor_data_t *motor_data,uint8_t *can_rx_data)
{
		motor_data->last_ecd 		= motor_data->ecd;
		motor_data->ecd      		= (uint16_t)(can_rx_data[0]<<8 |can_rx_data[1]);
		motor_data->speed_rpm		=	(uint16_t)(can_rx_data[2]<<8 |can_rx_data[3]);
		motor_data->give_current=	(uint16_t)(can_rx_data[4]<<8 |can_rx_data[5]);
		motor_data->temperature	=	can_rx_data[6];
}



/*****************************************[3]	数据处理*****************************************************************/
/**[3]					
  * brief					根据获取的3508编码器数据，设法计算出它总的转动的编码器角度
	*	param[out]		总角度结构体变量指针(因为要给结构体赋值)
	*	param[in]			从can接收电机数据的结构体，用值传递
	* postscript		只要转一圈编码器改变8192的都适用				*/
void calculate_motor3508_total_angle(motor_data_t *motor_data)
{
	if(motor_data->offset_flag == 0)
	{
		motor_data->offset_ecd=motor_data->ecd;
		motor_data->offset_flag=1;
	}
	else
	{
		if(motor_data->ecd - motor_data->last_ecd >  4096) motor_data->round_cnt--;
		if(motor_data->ecd - motor_data->last_ecd < -4096) motor_data->round_cnt++;
	}
	motor_data->total_ecd=motor_data->round_cnt * (int64_t)8192 + motor_data->ecd - motor_data->offset_ecd;
	
}
