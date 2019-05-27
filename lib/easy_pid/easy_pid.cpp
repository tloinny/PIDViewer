/**
 *@title PID controller class for Arduino
 *
 *@date on 2019-5-15
 *@author: tony-lin
 *@version 1.0.0
 *
 *@description: This class has been designed to use PID controller easyer.
 */
#include "easy_pid.h"

/**
 *@function initialize the PID controller
 *@param 
 *		Kp: Proportionality coefficient
 *		Ki: Integral coefficient
 *		Kd: Differential coefficient
 *		s_t: Sample time
 *		w_g: Windup guard
 *@return none
 */
PID_CONTROLLER::PID_CONTROLLER(float Kp, float Ki, float Kd, float s_t, float w_g)
{
	setPID(Kp, Ki, Kd);
	setSampleTime(s_t);
	setWindup(w_g);
	clear();
	CurrentTime = millis();	/* get current time */
	LastTime = CurrentTime;
}

/**
 *@function set the coefficients of PID controller
 *@param 
 *		p: Proportionality coefficient
 *		i: Integral coefficient
 *		d: Differential coefficient
 *@return none
 */
void PID_CONTROLLER::setPID(float p, float i, float d)
{
	kp = p;
	ki = i;
	kd = d;
}

/**
 *@function set the windup guard of PID controller
 *@param 
 *		windup: Windup guard
 *@return none
 */
void PID_CONTROLLER::setWindup(float windup)
{
	WindupGuard = windup;
}

/**
 *@function set the sample time of PID controller
 *@param 
 *		sample_time: sample time
 *@return none
 */
void PID_CONTROLLER::setSampleTime(float sample_time)
{
	SampleTime = sample_time;
}

/**
 *@function set the goal of PID controller
 *@param 
 *		goal: goal
 *@return none
 */
void PID_CONTROLLER::setGoal(float goal)
{
	Goal = goal;
}

/**
 *@function get the proportionality coefficients of PID controller
 *@param none
 *@return 
 *		kp: Proportionality coefficient
 */
float PID_CONTROLLER::getKp()
{
	return kp;
}

/**
 *@function get the integral coefficients of PID controller
 *@param none
 *@return 
 *		ki: Integral coefficient
 */
float PID_CONTROLLER::getKi()
{
	return ki;
}

/**
 *@function get the differential coefficients of PID controller
 *@param none
 *@return 
 *		kd: Differential coefficient
 */
float PID_CONTROLLER::getKd()
{
	return kd;
}

/**
 *@function get the windup guard of PID controller
 *@param none
 *@return 
 *		WindupGuard
 */
float PID_CONTROLLER::getWindup()
{
	return WindupGuard;
}

/**
 *@function initialize the other values of PID controller
 *@param none
 *@return 
 *		WindupGuard
 */
void PID_CONTROLLER::clear()
{
	Goal = 0;
	Output = 0;
	PTerm = 0;
	ITerm = 0;
	DTerm = 0;
	LastError = 0;
}

/**
 *@function Calculates PID value for given reference feedback
 *		u(t) = K_p*e(t) + K_i*(\int_{0}^{t} e(t)dt) + K_d*({de}/{dt})
 *@param 
 *		FeedbackValue
 *@return 
 *		Output
 */
float PID_CONTROLLER::update(float FeedbackValue)
{
	float error = Goal - FeedbackValue;	/* calculate the e(t) */
	
	CurrentTime = millis(); /* get current time */
	float delta_time = CurrentTime - LastTime;	/* calculate Δtime */
	float delta_error = error - LastError;	/* calculate Δerror */

	if (delta_time >= SampleTime)
	{
		PTerm = kp * error;	/* calculate Kp*e(t) */
		ITerm += error * delta_time;	/* calculate the approximate value of \int_{0}^{t} e(t)dt*/
			if (ITerm < -WindupGuard)	/* protect the ITerm value not to over the safe range */
			{
				ITerm = -WindupGuard;
			}
			else if (ITerm > WindupGuard)
			{
				ITerm = WindupGuard;
			}

			if (delta_time > 0)	/* don't make some mathematics mistake */
			{
				DTerm = delta_error / delta_time;
			}
		LastTime = CurrentTime;
		LastError = error;

		Output = PTerm + ki * ITerm + kd * DTerm;	/* calculate the value of output */
	}
	return Output;
}
