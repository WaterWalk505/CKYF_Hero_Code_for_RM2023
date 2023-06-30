#include "can_transmit.h"

#include "can.h"
#include "motor_can.h"

#include "Referee.h"

/**	[1]							
  * brief         	向超电发信息
	* postscript									*/
void send_power_information_to_Cap_through_CAN1(void)
{
	CAN_TxHeaderTypeDef CAN1_Capacitance_TxdHeader;
	uint8_t Capacitance_Given[8];
	uint32_t Capacitance_MailBox;
	CAN1_Capacitance_TxdHeader.StdId = 0x140;
	CAN1_Capacitance_TxdHeader.IDE   = CAN_ID_STD;
	CAN1_Capacitance_TxdHeader.RTR   = CAN_RTR_DATA;
	CAN1_Capacitance_TxdHeader.DLC   = 8;
	
	Capacitance_Given[0] = (Game_Robot_State.chassis_power_limit * 100) >> 0;
	Capacitance_Given[1] = (Game_Robot_State.chassis_power_limit * 100) >> 8;
	Capacitance_Given[2] = (int16_t)(Power_Heat_Data.chassis_power * 100) >> 0;
	Capacitance_Given[3] = (int16_t)(Power_Heat_Data.chassis_power * 100) >> 8;
	Capacitance_Given[4] = Power_Heat_Data.chassis_power_buffer * 100 >> 0;
	Capacitance_Given[5] = Power_Heat_Data.chassis_power_buffer * 100 >> 8;
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_Capacitance_TxdHeader, Capacitance_Given, &Capacitance_MailBox);
}


/**	[2]							
  * brief         	发一个float型数据
	* postscript									*/
uint8_t data_sending_can_send_data_watch[4];
void send_one_float_data_through_CAN2(float one_float_data)
{
	CAN_TxHeaderTypeDef data_sending_tx_message;
	uint8_t data_sending_can_send_data[4];
	uint32_t send_mail_box;
	
	data_sending_tx_message.StdId=0x162;
	data_sending_tx_message.RTR=CAN_RTR_DATA;
	data_sending_tx_message.IDE=CAN_ID_STD;
	data_sending_tx_message.DLC=0x04;
	
	memcpy(data_sending_can_send_data,&one_float_data,sizeof(data_sending_can_send_data));
	memcpy(data_sending_can_send_data_watch,&one_float_data,sizeof(data_sending_can_send_data));
	HAL_CAN_AddTxMessage(&hcan2,&data_sending_tx_message,data_sending_can_send_data,&send_mail_box);
}

/**	[3]							
  * brief         	向云台发裁判系统数据
	* postscript									*/
void send_referee_data_through_CAN2(void)
{
	CAN_TxHeaderTypeDef data_sending_tx_message;
	uint8_t data_sending_can_send_data[8];
	uint32_t send_mail_box;
	
	data_sending_tx_message.StdId=0x152;
	data_sending_tx_message.RTR=CAN_RTR_DATA;
	data_sending_tx_message.IDE=CAN_ID_STD;
	data_sending_tx_message.DLC=0x08;
	
	data_sending_can_send_data[0]=Power_Heat_Data.shooter_id1_42mm_cooling_heat >> 8;//枪口热量
	data_sending_can_send_data[1]=Power_Heat_Data.shooter_id1_42mm_cooling_heat & 0xff;
	data_sending_can_send_data[2]=Game_Robot_State.shooter_id1_42mm_cooling_limit >> 8;//枪口热量上限
	data_sending_can_send_data[3]=Game_Robot_State.shooter_id1_42mm_cooling_limit & 0xff;
	data_sending_can_send_data[4]=Game_Robot_State.shooter_id1_42mm_speed_limit >> 8;//弹速上限
	data_sending_can_send_data[5]=Game_Robot_State.shooter_id1_42mm_speed_limit & 0xff;
	data_sending_can_send_data[6]=Game_Robot_State.mains_power_shooter_output;
	data_sending_can_send_data[7]=Game_Robot_State.mains_power_chassis_output;
	
	HAL_CAN_AddTxMessage(&hcan2,&data_sending_tx_message,data_sending_can_send_data,&send_mail_box);
}
/**	[3]							
  * brief         	向云台发裁判系统数据,发的是机器人剩余血量
	* postscript									*/
void send_remain_HP_through_CAN2(void)
{
	CAN_TxHeaderTypeDef data_sending_tx_message;
	uint8_t data_sending_can_send_data[8];
	uint32_t send_mail_box;
	
	data_sending_tx_message.StdId=0x153;
	data_sending_tx_message.RTR=CAN_RTR_DATA;
	data_sending_tx_message.IDE=CAN_ID_STD;
	data_sending_tx_message.DLC=0x08;
	
	data_sending_can_send_data[0]=Game_Robot_State.remain_HP >> 8;//机器人剩余血量低八位
	data_sending_can_send_data[1]=Game_Robot_State.remain_HP & 0xff;//机器人剩余血量高八位
	data_sending_can_send_data[2]=Game_Robot_State.chassis_power_limit >> 8;//机器人底盘功率限制低八位
	data_sending_can_send_data[3]=Game_Robot_State.chassis_power_limit & 0xff;//机器人底盘功率限制高八位
	
	HAL_CAN_AddTxMessage(&hcan2,&data_sending_tx_message,data_sending_can_send_data,&send_mail_box);

}

void send_something_through_CAN2(void)
{
	CAN_TxHeaderTypeDef data_sending_tx_message;
	uint8_t data_sending_can_send_data[8];
	uint32_t send_mail_box;
	
	data_sending_tx_message.StdId=0x152;
	data_sending_tx_message.RTR=CAN_RTR_DATA;
	data_sending_tx_message.IDE=CAN_ID_STD;
	data_sending_tx_message.DLC=0x08;
	
	data_sending_can_send_data[0]=0x00;//int16_t数据的高八位
	data_sending_can_send_data[1]=0x01;//int16_t数据的低八位
	data_sending_can_send_data[2]=0x02;
	data_sending_can_send_data[3]=0x03;
	data_sending_can_send_data[4]=0x04;
	data_sending_can_send_data[5]=0x05;
	data_sending_can_send_data[6]=0x06;
	data_sending_can_send_data[7]=0x07;
	
	HAL_CAN_AddTxMessage(&hcan2,&data_sending_tx_message,data_sending_can_send_data,&send_mail_box);
}
