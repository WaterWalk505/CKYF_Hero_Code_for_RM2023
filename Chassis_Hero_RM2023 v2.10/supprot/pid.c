#include "pid.h"
#include "main.h"//NULL在里面有定义吧


	
		
/**
  * @brief          PID初始化函数，初始化PID的参数
  * @param[in]  		指针参数传递，要修改的pid结构体
	* @param[in]			PID本身属性参数：PID模式，PID的三个值
	* @param[in]			PID限幅
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
		if(	pid	==	NULL	||	PID	==	NULL)
		{
				return;
		}
		
		pid->mode=mode;
		
		pid->Kp=PID[0];
		pid->Ki=PID[1];
		pid->Kd=PID[2];
		
		pid->max_out=max_out;
		pid->max_iout=max_iout;
		
		pid->out		 	= 	pid->Pout 		= 	pid->Iout 		= 	pid->Dout = 0.0f;
		pid->Dbuf[0] 	= 	pid->Dbuf[1] 	= 	pid->Dbuf[2] 	= 	0.0f;
    pid->error[0] = 	pid->error[1] = 	pid->error[2] = 	0.0f;
}


/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针,指针参数传递
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid,	fp32 ref,	fp32 set)
{
		if(pid==NULL)
		{
			return 0.0f;
		}
	
		pid->error[2]	=	pid->error[1];
		pid->error[1]	=	pid->error[0];
		pid->Dbuf[2]	=	pid->Dbuf[1];
		pid->Dbuf[1]	=	pid->Dbuf[0];
		
		pid->set	=	set;
		pid->fdb	=	ref;
		pid->error[0]	=	set	-	ref;
		pid->Dbuf[0]	=	(pid->error[0] - pid->error[1]);
		
		if(pid->mode	==	PID_POSITION)
		{
			pid->Pout	=		pid->Kp	*	pid->error[0];
			pid->Iout	+=	pid->Ki	*	pid->error[0];
			pid->Dout	=		pid->Kd	*	(pid->error[0] - pid->error[1]);
			
			pid->Iout	=	pid->Iout >	pid->max_iout	?		pid->max_iout : pid->Iout;		//涓婁笅閮藉緱闄愬埗						
			pid->Iout	=	pid->Iout <	-pid->max_iout	?		-pid->max_iout : pid->Iout;
			pid->out	=	pid->Pout	+	pid->Iout	+	pid->Dout;
			pid->out	=	pid->out 	>	pid->max_out	?		pid->max_out 	: pid->out;		
			pid->out	=	pid->out 	<	-pid->max_out	?		-pid->max_out 	: pid->out;
		}
		
		if(pid->mode 	==	PID_DELTA)
		{
			pid->Pout	=		pid->Kp	*	(pid->error[0] - pid->error[1]);
			pid->Iout	=		pid->Ki	*	pid->error[0];
			pid->Dout	=		pid->Kd	*	(pid->error[0] - 2.0f * pid->error[1] + pid->error[2])	;
			
			pid->out	+=	pid->Pout	+	pid->Iout	+	pid->Dout;
			pid->out	=		pid->out 	>	pid->max_out	?		pid->max_out 	: pid->out;
			pid->out	=	pid->out 	<	-pid->max_out	?		-pid->max_out 	: pid->out;
		}

		return pid->out;
}



/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针，指针参数传递
  * @retval         none
  */
void pid_clear(pid_type_def *pid)
{
		if(pid	==	NULL)
		{
				return;
		}
		
		pid->set			=		pid->fdb			=		0.0f;
		pid->out		 	= 	pid->Pout 		= 	pid->Iout 		= 	pid->Dout = 0.0f;
		pid->Dbuf[0] 	= 	pid->Dbuf[1] 	= 	pid->Dbuf[2] 	= 	0.0f;
    pid->error[0] = 	pid->error[1] = 	pid->error[2] = 	0.0f;
		
}







