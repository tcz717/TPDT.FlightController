#include "PID.h"
#define PI 3.1415926

void PID_SetTarget(PID* pid,double value)
{
	pid->expect = value;
//	pid->out = 0;
//	pid->iv = 0;
//	pid->dv=0;
//	pid->ldv=0;
}
double PID_Update(PID* pid,double value, double dv)
{
	double p, i, d;
	pid->iv += (value - pid->expect);
	pid->iv=RangeValue(pid->iv,-360,+360);
	
	p = (value - pid->expect) * pid->p;
	i = pid->iv * pid->i;
	d = dv * pid->d;
	pid->outp=p;
	pid->outi=RangeValue(i,-100,+100);
	pid->outd=RangeValue(d,-500,+500);
	pid->out =RangeValue(p + i + d,-800,+800);
	pid->input=value;
	if (pid->out<1&&pid->out>-1)
		pid->out=0;
	return pid->out;
}
extern struct ahrs_t ahrs;
double PID_xUpdate(PID* pid,double value)
{
	double dv= (value - pid->input) / pid->dt;
	pid->dv= pid->dv + pid->filt_alpha * (dv-pid->dv);
	
	pid->outp = (value - pid->expect) * pid->p;
	pid->outi += (value - pid->expect)*pid->i*pid->dt;;
	pid->outd = pid->dv * pid->d;

	pid->outp=pid->outp;
	pid->outi=RangeValue(pid->outi,-200,+200);
	pid->outd=RangeValue(pid->outd,-500,+500);
	pid->out =RangeValue(pid->outp + pid->outi + pid->outd,-500,+500);
	pid->input=value;
	return pid->out;
}
double RangeValue(double value,double min,double max)
{
	if (value >= max)
		return max;
	if (value <= min)
		return min;
	return value;
}

void PID_Init(PID* pid,double p,double i,double d)
{
	pid->p=p;
	pid->i=i;
	pid->d=d;
	
	PID_SetTarget(pid,0);
	
	pid->out = 0;
	pid->iv = 0;
	pid->dv=0;
	pid->filt_alpha=1;
}

void PID_Set_Filt_Alpha(PID* pid,double dt,double filt_hz)
{
	double rc = 1/(2*PI*filt_hz);
    pid->filt_alpha = dt / (dt + rc);
	pid->dt=dt;
}
	
