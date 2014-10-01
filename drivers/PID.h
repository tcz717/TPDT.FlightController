#ifndef _PID
#define _PID

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

typedef struct
{
	double p;
	double i;
	double d;
	double ldv;
	double lv;
	double expect;
	double out;
	double outp;
	double outi;
	double outd;
	double iv;
	double dv;
}PID;


extern PID pitch_pid,roll_pid,yaw_pid;

void PID_SetTarget(PID*,double value);
double PID_Update(PID*,double value, double dv);
double RangeValue(double value,double min,double max);
void PID_Init(PID*,double p,double i,double d);

#endif
