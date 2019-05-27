/**
 *@title PID controller class for Arduino
 *
 *@date on 2019-5-15
 *@author: tony-lin
 *@version 1.0.0
 *
 *@description: This class has been designed to use PID controller easyer.
 */
#ifndef EASY_PID_H
#define EASY_PID_H

#include <Arduino.h>

class PID_CONTROLLER
{
public:
	PID_CONTROLLER(float Kp, float Ki, float Kd, float s_t, float w_g);
	void setPID(float kp, float ki, float kd);
	void setWindup(float windup);
	void setSampleTime(float sample_time);
	void setGoal(float goal);
	void clear();
	float getKp();
	float getKi();
	float getKd();
	float getWindup();
	float update(float FeedbackValue);

private:
	float kp;
	float ki;
	float kd;
	float Goal;
	float Output;
	float PTerm;
	float ITerm;
	float DTerm;
	float LastError;
	float WindupGuard;
	float SampleTime;
	int CurrentTime;
	int LastTime;
};
#endif