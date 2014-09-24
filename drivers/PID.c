#include "PID.h"

void PID_SetTarget(PID* pid,double value)
{
	pid->expect = value;
	pid->out = 0;
	pid->iv = 0;
	pid->dv=0;
	pid->ldv=0;
}
double PID_Update(PID* pid,double value, double dv)
{
	double p, i, d;
	pid->iv += (value - pid->expect);
	pid->iv=RangeValue(pid->iv,-300,+300);
//	dv=0.3*dv+0.7*pid->ldv;
	
	p = (value - pid->expect) * pid->p;
	i = pid->iv * pid->i;
	d = dv * pid->d;
	pid->outp=p;
	pid->outi=RangeValue(i,-300,+300);
	pid->outd=RangeValue(d,-300,+300);
	pid->out =RangeValue(p + i + d,-400,+400);
	pid->ldv=dv;
	pid->lv=value;
	if (pid->out<10&&pid->out>-10)
		pid->out=0;
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
	
