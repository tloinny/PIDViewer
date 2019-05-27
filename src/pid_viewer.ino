#include <easy_pid.h>
#include "AS5600.h"
#include <Servo.h>
#include <Wire.h>

AMS_5600 ams_5600;
Servo motor;
PID_CONTROLLER PID_contorller(0.2, 0, 0, 0.01, 20.0);
float feedback = 0;
float output = 0;

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

void setup()
{
	Serial.begin(115200);
	Wire.begin();
	ams_5600.setStartPosition(word(134.15399/0.087));
	ams_5600.setEndPosition(word(324.94498/0.087));
	PID_contorller.setGoal(90);
}

void loop()
{
	feedback = convertScaledAngleToDegrees(ams_5600.getScaledAngle());
	output = PID_contorller.update(feedback);
	Serial.println(output, DEC);
}
