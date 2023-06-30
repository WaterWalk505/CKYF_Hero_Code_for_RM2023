#include "can_receive.h"
#include "can.h"
#include "motor_can.h"
#include "bsp_uart.h"

/**	[1]							canRX�ص�����
  * brief         	canÿ������һ֡���ݽ�������жϺ��жϻص����������жϺ������洦����յ������ݣ�ʵʱ���µ������
	* postscript			CAN1:���̵��ID=1��2��3��4��CAN2����̨�����豸������						*/
extern motor_data_t chassis_motor_data[4];
extern int16_t hero_target_forward_speed_x,hero_target_right_speed_y,hero_target_rotation_speed_z;
extern uint8_t ChassisMode;
extern motor_data_t yaw_motor_data;

extern uint8_t UI_Capacitance ;   //����ʣ������
extern float Chassis_Capacitance;
extern float Chassis_CapPower;
extern uint8_t AutoAimingFlag;
extern uint16_t SpirallingFlag;
extern uint16_t FrictionFlag;
extern float INS_roll_for_UI;
extern float UI_Gimbal_Pitch;
extern uint8_t DownDialFlag;
uint8_t INS_roll_for_UI_temp[4];
uint8_t INS_pitch_for_UI_temp[4];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
	
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	
		/**(1) CAN1���յ��̵����� **/
		if(hcan->Instance==CAN1)
		{
			
			switch(rx_header.StdId)
			{
				/*(1.1)	���յ����ĸ����������*/
				case 0x201:get_motor3508_data(&chassis_motor_data[0],rx_data);break;
				case 0x202:get_motor3508_data(&chassis_motor_data[1],rx_data);break;
				case 0x203:get_motor3508_data(&chassis_motor_data[2],rx_data);break;
				case 0x204:get_motor3508_data(&chassis_motor_data[3],rx_data);break;
				default:break;
			}
			
				/* ������������ */
		if(rx_header.StdId == 0x130)
		{
			UI_Capacitance      = (((uint16_t)rx_data[1]) << 8 | rx_data[0]) / 32768.0f * 100.0f;
			Chassis_Capacitance = (float)((uint16_t)rx_data[1] << 8 | rx_data[0]) / 32768.0f;
	    Chassis_CapPower    = (float)((uint16_t)rx_data[3] << 8 | rx_data[2]) / 100.0f;
		}
			
		}
		
		
		/**(2) CAN2������̨������ **/
		if(hcan->Instance==CAN2)
		{
			switch(rx_header.StdId)
			{
				
				/*(2.1)	������̨ң����ҡ�˵�����*/
				case 0x150:						
				{
					ChassisMode=rx_data[0];
					hero_target_forward_speed_x=(((int16_t)rx_data[1]<<8)|rx_data[2]);
					hero_target_right_speed_y=(((int16_t)rx_data[3]<<8)|rx_data[4]);
					hero_target_rotation_speed_z=(((int16_t)rx_data[5]<<8)|rx_data[6]);
					break;
				}
				
				/*(2.2)	����yaw����������*/
				case 0x205:
				{
					get_motor6020_data(&yaw_motor_data,rx_data);
					break;
				}
					/*(2.3)	????UI???*/
				case 0x151:						
				{
					memcpy(&INS_roll_for_UI_temp,&rx_data,sizeof(INS_roll_for_UI_temp));//?????????A??????
					memcpy(&INS_roll_for_UI,&rx_data,sizeof(INS_roll_for_UI_temp));
					AutoAimingFlag=rx_data[4];
					SpirallingFlag=rx_data[5];
					FrictionFlag=rx_data[6];
					DownDialFlag=rx_data[7];
					break;
				}
				case 0x152:						
				{
					memcpy(&INS_pitch_for_UI_temp,&rx_data,sizeof(INS_pitch_for_UI_temp));//?????????A??????
					memcpy(&UI_Gimbal_Pitch,&rx_data,sizeof(INS_pitch_for_UI_temp));
                                           
					break;
				}
			}
			
		}
}
