#ifndef _PID
#define _PID
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

void PID_SetTarget(PID*,double value);
double PID_Update(PID*,double value, double dv);
double RangeValue(double value,double min,double max);

#endif
