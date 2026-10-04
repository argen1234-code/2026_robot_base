#ifndef PID_H
#define PID_H

#include "main.h"

#define LimitMax(input, max)	\
{                        			\
	if (input > max)      			\
	{                     			\
		input = max;       				\
	}                     			\
	else if (input < -max)			\
	{                     			\
		input = -max;      				\
	}                     			\
}

enum PID_MODE
{
	PID_POSITION = 0
};

typedef struct {
	
	uint8_t mode;

	double Target;
	double Actual;
	double Out;
	
	double Kp;
	double Ki;
	double Kd;
	
	double Error0;
	double Error1;
	double ErrorInt;
	double D_error;
	
	double OutMax;
	double max_iout;
	
	

} PID_t;


extern void PID_init(PID_t *pid, uint8_t mode, const double PID[3], double max_out, double max_iout);
extern double PID_Calculate(PID_t *pid, double ref, double set);

#endif






