#include "pid.h"
#include "main.h"//NULL�������ж����


	
		
/**
  * @brief          PID��ʼ����������ʼ��PID�Ĳ���
  * @param[in]  		ָ��������ݣ�Ҫ�޸ĵ�pid�ṹ��
	* @param[in]			PID�������Բ�����PIDģʽ��PID������ֵ
	* @param[in]			PID�޷�
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
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��,ָ���������
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
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
			
			pid->Iout	=	pid->Iout >	pid->max_iout	?		pid->max_iout : pid->Iout;		//上下都得限制						
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
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ�룬ָ���������
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







