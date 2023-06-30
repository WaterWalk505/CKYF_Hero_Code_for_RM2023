#include "referee_task.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "arm_math.h"

/* Private define ------------------------------------------------------------*/
#define Referee_FIFOInit fifo_s_init
#define Max(a,b) ((a) > (b) ? (a) : (b))

//#define ROBOT_ID_MYSELF Robot_ID_Blue_Hero//蓝英雄
#define ROBOT_ID_MYSELF Robot_ID_Red_Hero//红英雄

/* Private variables ---------------------------------------------------------*/
TaskHandle_t RefereeTask_Handle;

/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;

#define sin15 0.2588f
#define cos15 0.9659f

/* 动态UI数据变量 */
uint8_t UI_AutoAim_Flag = 0;    //是否开启自瞄标志位
float   UI_Kalman_Speed = 0;    //卡尔曼预测速度
float   UI_Gimbal_Pitch = 0.0f; //云台Pitch轴角度
float   UI_Gimbal_Yaw   = 0.0f; //云台Yaw轴角度
float   UI_Gimbal_Roll  = 0.0f; //云台Roll轴角度
uint8_t UI_Capacitance  = 10;   //电容剩余容量
uint8_t UI_fric_is_on   = 0;    //摩擦轮是否开启
//float   UI_Gimbal_Pitch = 0.0f; //云台Pitch轴角度
uint8_t UI_fric_is_on;		//摩擦轮是否开启
uint8_t UI_Capacitance;  //电容剩余容量
extern uint8_t ChassisMode;
uint8_t UI_Capacitance_Color;

float UI_Capacitance_Value=100.00f;	//超级电容剩余容量百分比
float INS_roll_for_UI;

_Bool FrictionChange		=0;
_Bool FrictionFlag			=0;
_Bool FrictionFlagPast	=0;
_Bool AutoAimingChange	=0;
_Bool AutoAimingFlag		=0;
_Bool AutoAimingFlagPast=0;
_Bool SpirallingChange	=0;
_Bool SpirallingFlag		=0;
_Bool SpirallingFlagPast=0;
_Bool DownDialChange	=0;
_Bool DownDialFlag		=0;
_Bool DownDialFlagPast=0;

unsigned int referee_i=0;

int test1;
int test2;
int test3;
int test4;
int test_flag=0;
int x,y;
//extern int16_t Chassis_TopRotate_Scale; //小陀螺转速比例				//移植代码先暂时注释掉
//void RefereeTask(void const * argument)
void referee_task(void const * argument)
{

	/* 裁判系统初始化 */
	Referee_StructInit();
	Referee_UARTInit(Referee_Buffer[0], Referee_Buffer[1], REFEREE_USART_RX_BUF_LENGHT);
	Referee_FIFOInit(&Referee_FIFO, Referee_FIFO_Buffer, REFEREE_FIFO_BUF_LENGTH);
	vTaskDelay(300);
	uint8_t test=0;
	
	while(1)
	{
//			Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);//读取并解析裁判系统数据
		test++;
		/* 静态UI加载 */
		
		UI_Draw_Line	(&UI_Graph7.Graphic[0], "001",	UI_Graph_Add, 0,	UI_Color_Cyan,	2,  660,	130,  1260,	130);		//超电能量槽1
		UI_Draw_Line	(&UI_Graph7.Graphic[1], "002",	UI_Graph_Add, 0,	UI_Color_Cyan,	2,  660,	130,  660,  170);		//超电能量槽2
		UI_Draw_Line	(&UI_Graph7.Graphic[2], "003",	UI_Graph_Add, 0,	UI_Color_Cyan,	2,  1260,	130,  1260,	170);		//超电能量槽3
		UI_Draw_Line	(&UI_Graph7.Graphic[3], "004",	UI_Graph_Add, 0,	UI_Color_Cyan,	2,  660,	170,  1260,	170);		//超电能量槽4
		UI_Draw_Line	(&UI_Graph7.Graphic[4], "005",	UI_Graph_Add, 0,	UI_Color_Main,	0,	test1,test2,test3,test4);						//测试线
		UI_Draw_Line	(&UI_Graph7.Graphic[5], "006",	UI_Graph_Add, 0,	UI_Color_Main,	0,	test1,test2,test3,test4);						//测试线
		UI_Draw_Line	(&UI_Graph7.Graphic[6], "007",	UI_Graph_Add, 0,	UI_Color_Main,	0,	test1,test2,test3,test4);						//测试线
//		UI_Draw_Line	(&UI_Graph7.Graphic[4], "005",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  870,	730,  1050,	730);			//二米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[5], "006",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  880,	720,  1040, 720);			//五米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[6], "007",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  890,  690,  1030, 690);	//六米线
		UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
		vTaskDelay(131);
//		
//		UI_Draw_Line	(&UI_Graph7.Graphic[0], "008",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  900,		670,  	1020,  	670);			//七米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[1], "009",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  910,		635,  	1010,		635 );		//八米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[2], "010",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  920,	613,  	1000,  613);				//九米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[3], "011",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  930,	585,  	990,  585);		//十米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[4], "012",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  930,		565,  	990,		565);			//十一米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[5], "013",	UI_Graph_Add, 0,	UI_Color_Cyan,	1,  930,		523,  	990,		523);			//十二米线
//		UI_Draw_Line	(&UI_Graph7.Graphic[6], "014",	UI_Graph_Add, 0,	UI_Color_Main,	2,  test1,test2,test3,test4);	//测试线
//		UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
//		vTaskDelay(131);
//	
//		UI_Draw_Int	(&UI_Graph7.Graphic[0], "015",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,	1,		1052,  	732,  2);	//二米线标识
//		UI_Draw_Int	(&UI_Graph7.Graphic[1], "016",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,  1,		1042,  	722,	5);		//五米线标识
//		UI_Draw_Int	(&UI_Graph7.Graphic[2], "017",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,	 1, 1032,  	692,  6);		//六米线标识
//		UI_Draw_Int	(&UI_Graph7.Graphic[3], "018",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,  1,	  1022,  	672,  7);		//七米线标识
//		UI_Draw_Int	(&UI_Graph7.Graphic[4], "019",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,  1,		1012,  	637,	8);		//八米线标识
//		UI_Draw_Int	(&UI_Graph7.Graphic[5], "020",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,  1,		1002,  	615,	9);		//九米线标识
//		UI_Draw_Int	(&UI_Graph7.Graphic[6], "021",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,  1,    992,    587,  10);	//十米线标识
//		UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
//		vTaskDelay(131);
//		UI_Draw_Int		(&UI_Graph7.Graphic[0], "022",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,	1,	992,  	567,	11);	//十一米线标识
//		UI_Draw_Int	  (&UI_Graph7.Graphic[1], "023",	UI_Graph_Add, 0,	UI_Color_Cyan,	10,	1,	992,  	525,	12);	//十二米密位线
//		UI_Draw_Int	  (&UI_Graph7.Graphic[2], "024",	UI_Graph_Add, 0,	UI_Color_Cyan,	0,	0,	0,  	  0,	0	);
//		UI_Draw_Int	  (&UI_Graph7.Graphic[3], "025",	UI_Graph_Add, 0,	UI_Color_Cyan,	0,	0,	0,  	  0,	0	);
//		UI_Draw_Int	  (&UI_Graph7.Graphic[4], "026",	UI_Graph_Add, 0,	UI_Color_Cyan,	0,	0,	0,  		0,	0	);
//		UI_Draw_Int	  (&UI_Graph7.Graphic[5], "027",	UI_Graph_Add, 0,	UI_Color_Cyan,	0,	0,	0,  		0,	0	);
//		UI_Draw_Int	  (&UI_Graph7.Graphic[6], "028",	UI_Graph_Add, 0,	UI_Color_Cyan,	0,  0,  0,   	  0 , 0);
//		UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
//		vTaskDelay(131);
		
		
//		UI_Draw_Arc		(&UI_Graph7.Graphic[0], "022",	UI_Graph_Add, 0,	UI_Color_Green,	165,180,1,960,840,100*sin15	,100*cos15);	
//		UI_Draw_Arc	  (&UI_Graph7.Graphic[1], "023",	UI_Graph_Add, 0,	UI_Color_Green,	165,180,1,960,840,200*sin15	,200*cos15);		//二米密位线
//		UI_Draw_Arc	  (&UI_Graph7.Graphic[2], "024",	UI_Graph_Add, 0,	UI_Color_Green,	165,180,1,960,840,300*sin15	,300*cos15);
//		UI_Draw_Arc	  (&UI_Graph7.Graphic[3], "025",	UI_Graph_Add, 0,	UI_Color_Green,	165,180,1,960,840,400*sin15	,400*cos15);
//		UI_Draw_Arc	  (&UI_Graph7.Graphic[4], "026",	UI_Graph_Add, 0,	UI_Color_Green,	165,180,1,960,840,500*sin15	,500*cos15);
//		UI_Draw_Arc	  (&UI_Graph7.Graphic[5], "027",	UI_Graph_Add, 0,	UI_Color_Green,	165,180,1,960,840,600*sin15	,600*cos15);
//		UI_Draw_Arc	  (&UI_Graph7.Graphic[6], "028",	UI_Graph_Add, 0,	UI_Color_Green,	165,180,1,960,840,700*sin15	,700*cos15);
//		UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
//		vTaskDelay(131);
		
		/* 静态文字UI */
		UI_Draw_String(&UI_String.String,			"101",	UI_Graph_Add,	0,	UI_Color_Cyan,	19, 8, 	2,  50, 800,	"Friction");		//摩擦轮
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		UI_Draw_String(&UI_String.String,			"102",	UI_Graph_Add,	0,	UI_Color_Cyan,	19,	10,	2,	50,	825,	"AutoAiming");	//自动瞄准
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		UI_Draw_String(&UI_String.String,			"103",	UI_Graph_Add,	0,	UI_Color_Cyan,	19,	10,	2,	50,	850,	"Spiralling");	//小陀螺
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		UI_Draw_String(&UI_String.String,			"104",	UI_Graph_Add,	0,	UI_Color_Cyan,	19,	11,	2,	50,	875,	"ChassisMode");	//底盘跟随云台
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		UI_Draw_String(&UI_String.String,			"105",	UI_Graph_Add,	0,	UI_Color_Cyan,	19,	11,	2,	50,	900,	"DownDial");	//下拨弹盘
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
//		UI_Draw_String(&UI_String.String,			"104",	UI_Graph_Add,	0,	UI_Color_Cyan,	19,	11,	2,	50,	875,	"GimbalPitch");	//云台角度
//		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
//		vTaskDelay(131);
//		UI_Draw_String(&UI_String.String,			"105",	UI_Graph_Add,	0,	UI_Color_Cyan,	19,	11,	2,	50,	900,	"Capacitance");	//超级电容
//		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
//		vTaskDelay(131);
		
		/* 动态UI预加载 */
		
		UI_Draw_String(&UI_String.String,			"208",	UI_Graph_Add,	0,	UI_Color_Orange,	19,	11,		3,	350,	800,	"Off        ");	//摩擦轮状态
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		
		UI_Draw_String(&UI_String.String,			"209",	UI_Graph_Add,	0,	UI_Color_Orange,	19,	11,		3,	350,	825,	"Off        ");	//自瞄状态
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		
		UI_Draw_String(&UI_String.String,			"210",	UI_Graph_Add,	0,	UI_Color_Orange,	19,	11,		3,	350,	850,	"Off        ");	//小陀螺状态
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		
		UI_Draw_String(&UI_String.String,			"211",	UI_Graph_Add,	0,	UI_Color_Orange,	19,	11,		3,	350,	900,	"Off        ");	//下拨弹盘状态
		UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
		vTaskDelay(131);
		
		UI_Draw_Line	(&UI_Graph7.Graphic[0], "201",	UI_Graph_Add, 0,	UI_Color_Yellow,				2,  580,	0,  818,	360);														//车体边界线1
		UI_Draw_Line	(&UI_Graph7.Graphic[1], "202",	UI_Graph_Add, 0,	UI_Color_Yellow,				2,  1340,	0,  1102,	360);														//车体边界线2
		UI_Draw_Line	(&UI_Graph7.Graphic[2], "203",	UI_Graph_Add, 0,	UI_Color_Yellow,				0,  1500,	400,  1900,	0);															//车体边界线3
		UI_Draw_Line	(&UI_Graph7.Graphic[3], "204",	UI_Graph_Add, 0,	UI_Capacitance_Color,	40,	660,	150,  660+6.0f*UI_Capacitance_Value,  150);	//超电能量条
		UI_Draw_Float	(&UI_Graph7.Graphic[4], "205",	UI_Graph_Add, 0,	UI_Capacitance_Color,	19,	1,		3,  	1270,	160,	UI_Capacitance_Value);		//超电能量值
		UI_Draw_Float	(&UI_Graph7.Graphic[5], "206",	UI_Graph_Add, 0,	UI_Color_Main,				19,	1, 		3, 		1360,	630,	-1*UI_Gimbal_Pitch);					//Pitch轴角度
		UI_Draw_Line	(&UI_Graph7.Graphic[6], "207",	UI_Graph_Add, 0,	UI_Color_Green,	      1,	955,	240, 955,  840);														//roll角度线
//		UI_Draw_Line	(&UI_Graph7.Graphic[6], "207",	UI_Graph_Add, 0,	UI_Color_Main,				2,  test1,test2,test3,test4);													//测试线
		UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
		vTaskDelay(131);
		UI_Draw_Float (&UI_Graph7.Graphic[0],	"212",	UI_Graph_Add,	0,	UI_Color_Orange,	   19,	1,   3,    350,  875,	ChassisMode);	//底盘跟随云台状态									//测试线
		UI_Draw_Float	(&UI_Graph7.Graphic[1], "213",	UI_Graph_Add, 0,	UI_Color_Main,				2,	3, 		3, 		960,	225,	INS_roll_for_UI*180/3.14159f);					//Roll轴角度
		UI_Draw_Line	(&UI_Graph7.Graphic[2], "217",	UI_Graph_Add, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);												//测试线
		UI_Draw_Line	(&UI_Graph7.Graphic[3], "217",	UI_Graph_Add, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
		UI_Draw_Line	(&UI_Graph7.Graphic[4], "217",	UI_Graph_Add, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
		UI_Draw_Line	(&UI_Graph7.Graphic[5], "217",	UI_Graph_Add, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
		UI_Draw_Line	(&UI_Graph7.Graphic[6], "217",	UI_Graph_Add, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
		UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
		vTaskDelay(131);
		
		for(referee_i=0;referee_i<100;referee_i++)
		{
		
			/* 动态UI变量刷新 */
			
//			SpirallingFlag = Chassis_TopRotate_Scale;								//移植代码暂时先注释掉
			
			UI_Capacitance_Value=UI_Capacitance;
			
//			FrictionFlag=UI_fric_is_on;
			
			if(FrictionFlag!=FrictionFlagPast)
				FrictionChange=1;
			FrictionFlagPast	=FrictionFlag;
			if(AutoAimingFlag!=AutoAimingFlagPast)
				AutoAimingChange=1;
			AutoAimingFlagPast=AutoAimingFlag;
			if(SpirallingFlag!=SpirallingFlagPast )
				SpirallingChange=1;
			SpirallingFlagPast=SpirallingFlag;
			if(DownDialFlag!=DownDialFlagPast)
				DownDialChange=1;
			DownDialFlagPast=DownDialFlag;
			/* 超电颜色更改 */
			
			if(UI_Capacitance_Value>=50)
				UI_Capacitance_Color=UI_Color_Green;
			else if(UI_Capacitance_Value>=25)
				UI_Capacitance_Color=UI_Color_Yellow;
			else
				UI_Capacitance_Color=UI_Color_Orange;
			
			/* 动态UI刷新 */
//			UI_Draw_Line	(&UI_Graph7.Graphic[3], "207",	UI_Graph_Add, 0,	UI_Color_Green,	 19,	50,	500,  50,  650);	//超电能量条
			/* 未开启小陀螺 */
			if(SpirallingFlag==0)
			{
				UI_Draw_Line	(&UI_Graph7.Graphic[0], "201",	UI_Graph_Change, 0,	UI_Color_Yellow,				2,  580,	0,  818,	360);														//车体边界线1
				UI_Draw_Line	(&UI_Graph7.Graphic[1], "202",	UI_Graph_Change, 0,	UI_Color_Yellow,				2,  1340,	0,  1102,	360);														//车体边界线2
				UI_Draw_Line	(&UI_Graph7.Graphic[2], "203",	UI_Graph_Change, 0,	UI_Color_Main,				0,	1500,	400,  1900, 0);															//车体边界线3
				UI_Draw_Line	(&UI_Graph7.Graphic[3], "204",	UI_Graph_Change, 0,	UI_Capacitance_Color,	40,	660,	150,  660+6.0f*UI_Capacitance_Value,  150);	//超电能量条
				UI_Draw_Float	(&UI_Graph7.Graphic[4], "205",	UI_Graph_Change, 0,	UI_Capacitance_Color,	19,	1,		3,  	1270,	160,	UI_Capacitance_Value);		//超电能量值
				UI_Draw_Float	(&UI_Graph7.Graphic[5], "206",	UI_Graph_Change, 0,	UI_Color_Main,				19,	1, 		3, 		1360,	630,	-1*UI_Gimbal_Pitch);					//Pitch轴角度												//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[6], "207",	UI_Graph_Change, 0,	UI_Color_Green,	      1,	955+150*tan(INS_roll_for_UI-0.05f),	240, 955,  840);  //roll角度线
				UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
				vTaskDelay(131);
				UI_Draw_Float (&UI_Graph7.Graphic[0],	"212",	UI_Graph_Change,	0,	UI_Color_Orange,	   		19,	1,   3,    350,  875,	ChassisMode);	//底盘跟随云台状态									/
				UI_Draw_Float	(&UI_Graph7.Graphic[1], "213",	UI_Graph_Change, 0,	UI_Color_Main,				19,	3, 		3, 	960,855,	INS_roll_for_UI*180/3.14159f);					//Pitch轴角度
				UI_Draw_Line	(&UI_Graph7.Graphic[2], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);						//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[3], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[4], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[5], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[6], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
				vTaskDelay(131);
			}
			/* 开启小陀螺 */
			else if(SpirallingFlag==1)
			{
				UI_Draw_Arc		(&UI_Graph7.Graphic[0], "201",	UI_Graph_Change, 0,	UI_Color_Main,				270,90,		0,  	800,	0,	1000,	500);									//车体边界线
				UI_Draw_Line	(&UI_Graph7.Graphic[1], "202",	UI_Graph_Change, 0,	UI_Color_Main,				0,  0,		0,  	0,  	0);															//清空
				UI_Draw_Line	(&UI_Graph7.Graphic[2], "203",	UI_Graph_Change, 0,	UI_Color_Main,				0,  0,		0,  	0,  	0);															//清空
				UI_Draw_Line	(&UI_Graph7.Graphic[3], "204",	UI_Graph_Change, 0,	UI_Capacitance_Color,	40,	660,	150,  660+6.0f*UI_Capacitance_Value,  150);	//超电能量条
				UI_Draw_Float	(&UI_Graph7.Graphic[4], "205",	UI_Graph_Change, 0,	UI_Capacitance_Color,	19,	1,		3,  	1270,	160,	UI_Capacitance_Value);		//超电能量值
				UI_Draw_Float	(&UI_Graph7.Graphic[5], "206",	UI_Graph_Change, 0,	UI_Color_Main,				19,	1, 		3, 		1360,	630,	-1*UI_Gimbal_Pitch);					//Pitch轴角度
//				UI_Draw_Line	(&UI_Graph7.Graphic[6], "207",	UI_Graph_Change, 0,	UI_Color_Main,				2,  test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[6], "207",	UI_Graph_Change, 0,	UI_Color_Green,	      1,	955+150*tan(INS_roll_for_UI-0.05f),	240, 955,  840);	//roll角度线
				UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
				vTaskDelay(131);
				UI_Draw_Float (&UI_Graph7.Graphic[0],	"212",	UI_Graph_Change,	0,	UI_Color_Orange,	  19,	1,    3,    350,  875,	ChassisMode);	//底盘跟随云台状态									/
				UI_Draw_Float	(&UI_Graph7.Graphic[1], "213",	UI_Graph_Change, 0,	UI_Color_Main,19,	3, 		3, 	960,925,	INS_roll_for_UI*180/3.14159f);					//Pitch轴角度				//Pitch轴角度
				UI_Draw_Line	(&UI_Graph7.Graphic[2], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[3], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[4], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[5], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_Draw_Line	(&UI_Graph7.Graphic[6], "217",	UI_Graph_Change, 0,	UI_Color_Main,				2,	test1,test2,test3,test4);													//测试线
				UI_PushUp_Graphs(7,&UI_Graph7,ROBOT_ID_MYSELF);
				vTaskDelay(131);
			}
			
			/* 摩擦轮状态改变 */
			if(FrictionChange==1)
			{
				if(FrictionFlag)
				{
					UI_Draw_String(&UI_String.String,	"208",	UI_Graph_Change,	0,	UI_Color_Green,		19,	11,	3,	350,	800,	"On         ");
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					FrictionChange=0;
					vTaskDelay(131);
				}
				else
				{
					UI_Draw_String(&UI_String.String,	"208",	UI_Graph_Change,	0,	UI_Color_Orange,	19,	11,	3,	350,	800,	"Off        ");
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					FrictionChange=0;
					vTaskDelay(131);
				}
			}
			
			/* 自瞄状态改变 */
			if(AutoAimingChange==1)
			{
				if(AutoAimingFlag)
				{
					UI_Draw_String(&UI_String.String,	"209",	UI_Graph_Change,	0,	UI_Color_Green,		19,	11,	3,	350,	825,	"On         ");
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					AutoAimingChange=0;
					vTaskDelay(131);
				}
				else
				{
					UI_Draw_String(&UI_String.String,	"209",	UI_Graph_Change,	0,	UI_Color_Orange,	19,	11,	3,	350,	825,	"Off        ");
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					AutoAimingChange=0;
					vTaskDelay(131);
				}
			}
			
			/* 小陀螺状态改变 */
			if(SpirallingChange==1)
			{
				if(SpirallingFlag)
				{
					UI_Draw_String(&UI_String.String,	"210",	UI_Graph_Change,	0,	UI_Color_Green,		19,	11,	3,	350,	850,	"On         ");
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					SpirallingChange=0;
					vTaskDelay(131);
				}
				else
				{
					UI_Draw_String(&UI_String.String,	"210",	UI_Graph_Change,	0,	UI_Color_Orange,	19,	11,	3,	350,	850,	"Off        ");
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					SpirallingChange=0;
					vTaskDelay(131);
				}
			}
			/*下拨单盘状态改变*/		
			if(DownDialChange==1)
			{
				if(DownDialFlag)
				{
					UI_Draw_String(&UI_String.String,			"211",	UI_Graph_Add,	0,	UI_Color_Orange,	19,	11,		3,	350,	900,	"On        ");	//下拨弹盘状态
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					SpirallingChange=0;
					vTaskDelay(131);

				}
				else
				{
					UI_Draw_String(&UI_String.String,			"211",	UI_Graph_Add,	0,	UI_Color_Orange,	19,	11,		3,	350,	900,	"Off        ");	//下拨弹盘状态
					UI_PushUp_String(&UI_String, ROBOT_ID_MYSELF);
					SpirallingChange=0;
					vTaskDelay(131);
				}
			}
			/* 动态UI增加 */

			/* 动态UI删除 */
			
		}
	}
}

//void USART6_IRQHandler_0(void)
//{
//    static volatile uint8_t res;
//    if(USART6->SR & UART_FLAG_IDLE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart6);

//        static uint16_t this_time_rx_len = 0;

//        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(huart6.hdmarx);
//            fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[0], this_time_rx_len);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(huart6.hdmarx);
//            fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
//        }
//    }
//}

void Referee_IRQHandler(void)
{
	if(Referee_UART.Instance->SR & UART_FLAG_RXNE)
	{
	  __HAL_UART_CLEAR_IDLEFLAG(&Referee_UART);
	}
	else if(Referee_UART.Instance->SR & UART_FLAG_IDLE)
	{
		static uint16_t Size = 0;
		
		/* 清空标志位 */
		__HAL_UART_CLEAR_IDLEFLAG(&Referee_UART);
		
		if ((Referee_UART.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* 重置DMA并切换缓冲区 */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
			Size = REFEREE_USART_RX_BUF_LENGHT - Referee_UART.hdmarx->Instance->NDTR;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);
			
			/* 将数据添加到队列 */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[0], Size);
		}
		else
		{
			/* 重置DMA并切换缓冲区 */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
			Size = REFEREE_USART_RX_BUF_LENGHT - Referee_UART.hdmarx->Instance->NDTR;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);
			
			/* 将数据添加到队列 */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[1], Size);
		}
		return;
	}
	
	HAL_UART_IRQHandler(&Referee_UART);
}

void Referee_Usart_Unpack(void const * argument){
	vTaskDelay(1200);
	while(1){
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		vTaskDelay(5);
	}
}
