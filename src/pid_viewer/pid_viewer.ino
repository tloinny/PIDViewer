#include <FlexiTimer2.h>
#include <easy_pid.h>
#include "AS5600.h"
#include <Wire.h>

AMS_5600 ams_5600;
PID_CONTROLLER PID_contorller(1000000, 0, 0, 0.001, 10.0);
float feedback = 0;
float output = 0;
int throttle = 0;
int get_p = 0;

/**
 *@Function: convertScaledAngleToDegrees
 * In: angle data from AMS_5600::getScaledAngle
 * Out: human readable degrees as float
 * Description: takes the scaled angle and calculates
 * float value in degrees.
 */
float convertScaledAngleToDegrees(word newAngle)
{
  word startPos = ams_5600.getStartPosition();
  word endPos = ams_5600.getEndPosition();
  word maxAngle = ams_5600.getMaxAngle();

  float multipler = 0;

  /* max angle and end position are mutually exclusive*/
  if(maxAngle >0)
  {
    if(startPos == 0)
      multipler = (maxAngle*0.0878)/4096;
    else  /*startPos is set to something*/
      multipler = ((maxAngle*0.0878)-(startPos * 0.0878))/4096;
  }
  else
  {
    if((startPos == 0) && (endPos == 0))
      multipler = 0.0878;
    else if ((startPos > 0 ) && (endPos == 0))
      multipler = ((360 * 0.0878) - (startPos * 0.0878)) / 4096;
    else if ((startPos == 0 ) && (endPos > 0))
      multipler = (endPos*0.0878) / 4096;
    else if ((startPos > 0 ) && (endPos > 0))
      multipler = ((endPos*0.0878)-(startPos * 0.0878))/ 4096;
  }
  return (newAngle * multipler);
}

void Event()
{
  digitalWrite(3,HIGH);
  delayMicroseconds(throttle);	/* The stepper motor will be faster if this value become smaller */
  digitalWrite(3,LOW);
}

void setup()
{
	FlexiTimer2::set(3, 1.0/10000, Event);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	FlexiTimer2::start();
	Wire.begin();
	Serial.begin(115200);
	ams_5600.setStartPosition(word(0/0.087));
	ams_5600.setEndPosition(word(355/0.087));
	PID_contorller.setGoal(180);
	Serial.println("ready");
  throttle=1000;
}

void loop()
{
  Serial.print("angle: ");
  Serial.print(convertScaledAngleToDegrees(ams_5600.getScaledAngle()), DEC);
  Serial.print("  ");
	feedback = convertScaledAngleToDegrees(ams_5600.getScaledAngle());
	output = PID_contorller.update(feedback);	/* Stepper motor should run faster if this value become lager */
	Serial.print("output: ");
	Serial.print(output, DEC);
	Serial.println("  ");
    if(output > 0.1)
    {
      FlexiTimer2::start();
		  digitalWrite(4, HIGH);
		  //throttle = map(output*100, 0, 100, 0, 9999);
		  throttle -= output;
    }else if(output < -0.1)
    {
      FlexiTimer2::start();
		  digitalWrite(4, LOW);
		  //throttle = map(output*100, -100, 0, -3, 9999);
		  throttle -= output;
    }
    else
    {
		  FlexiTimer2::stop();
    }
}
