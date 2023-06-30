#ifndef CAN_TRANSMIT_H
#define CAN_TRANSMIT_H

void send_something_through_CAN2(void);
void send_referee_data_through_CAN2(void);
void send_one_float_data_through_CAN2(float one_float_data);
void send_power_information_to_Cap_through_CAN1(void);
void send_remain_HP_through_CAN2(void);
#endif
