#include "motor_can.h"
#include "can.h"
#include "chassis_motor.h"

/*****************************************[1]	����Э��*****************************************************************/

/**	[1.1]					 	
  * brief	  	     ��������Ŀ�����ֵ
  * param[in]      motor_current[0]:ID=1	����3508���	����
	*															[1]:ID=2	����3508���	����		
	*															[2]:ID=3	����3508���	����		
	*															[3]:ID=4	����3508���	����			
	* postscript		 ͨ�ţ�CAN1 	���Э�飺ID��1��4��3508���Э��*/
int16_t motor_3508current_on_CAN1_setting[4];//ͳһ��ֵ��ͳһ����
void send_motor_3508current_through_CAN1(int16_t motor1_current,int16_t motor2_current,int16_t motor3_current,int16_t motor4_current)
{
	CAN_TxHeaderTypeDef motor3508_current_setting_tx_message;
	uint8_t motor3508_current_setting_can_send_data[8];
	uint32_t send_mail_box;
	
	motor3508_current_setting_tx_message.StdId=0x200;//ID��1��4
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




/*****************************************[2]	����Э��*****************************************************************/


/**	[2.1]					 ��ȡ�����������
  * brief          ��������ͨ��can���������ݣ�����3508���ĸ�ʽ��������motor_data_t�ͽṹ����
  * param[in]      motor_data_t *motor_data��		�������λ�á��ٶȡ���������Ϣ��������ĵ�����ݷ�������ṹ������
  * param[in]      uint8_t *can_rx_data��				can���յ��ĵ��������ԭʼ����
	* postscript		 ��canRX�ص���������ʹ��																				*/
void get_motor3508_data(motor_data_t *motor_data,uint8_t *can_rx_data)
{
		motor_data->last_ecd 		= motor_data->ecd;
		motor_data->ecd      		= (uint16_t)(can_rx_data[0]<<8 |can_rx_data[1]);
		motor_data->speed_rpm		=	(uint16_t)(can_rx_data[2]<<8 |can_rx_data[3]);
		motor_data->give_current=	(uint16_t)(can_rx_data[4]<<8 |can_rx_data[5]);
		motor_data->temperature	=	can_rx_data[6];
}

/**	[2.2]					 ��ȡ�����������
  * brief          ��������ͨ��can���������ݣ�����6020���ĸ�ʽ��������motor_data_t�ͽṹ����
  * param[in]      motor_data_t *motor_data��		�������λ�á��ٶȡ���������Ϣ��������ĵ�����ݷ�������ṹ������
  * param[in]      uint8_t *can_rx_data��				can���յ��ĵ��������ԭʼ����
	* postscript		 ��canRX�ص���������ʹ��																				*/
motor_data_t yaw_motor_data;
void get_motor6020_data(motor_data_t *motor_data,uint8_t *can_rx_data)
{
		motor_data->last_ecd 		= motor_data->ecd;
		motor_data->ecd      		= (uint16_t)(can_rx_data[0]<<8 |can_rx_data[1]);
		motor_data->speed_rpm		=	(uint16_t)(can_rx_data[2]<<8 |can_rx_data[3]);
		motor_data->give_current=	(uint16_t)(can_rx_data[4]<<8 |can_rx_data[5]);
		motor_data->temperature	=	can_rx_data[6];
}



/*****************************************[3]	���ݴ���*****************************************************************/
/**[3]					
  * brief					���ݻ�ȡ��3508���������ݣ��跨��������ܵ�ת���ı������Ƕ�
	*	param[out]		�ܽǶȽṹ�����ָ��(��ΪҪ���ṹ�帳ֵ)
	*	param[in]			��can���յ�����ݵĽṹ�壬��ֵ����
	* postscript		ֻҪתһȦ�������ı�8192�Ķ�����				*/
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
