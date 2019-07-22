#include <easy_pid.h>
#include "AS5600.h"
#include <Wire.h>
#include "A4988.h"

AMS_5600 ams_5600;
PID_CONTROLLER PID_contorller(1, 0, 0, 0.0001, 10.0);
float feedback = 0;
float output = 0;
int throttle = 0;
int get_p = 0;

bool arrive = 0;

#define MOTOR_STEPS 200
#define Micro_step 1
#define DIR 4
#define STEP 3
#define MS1 0
#define MS2 0
#define MS3 0
A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

int set_point = 270;

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
	Wire.begin();
	Serial.begin(115200);
	ams_5600.setStartPosition(word(0/0.087));
	ams_5600.setEndPosition(word(355/0.087));
	PID_contorller.setGoal(set_point);
  stepper.begin(1, 1);
	Serial.println("ready");
}

void loop()
{
  Serial.print("angle: ");
  Serial.print(convertScaledAngleToDegrees(ams_5600.getScaledAngle()), DEC);
  Serial.print("  ");
	feedback = convertScaledAngleToDegrees(ams_5600.getScaledAngle());
  if(!(abs(feedback-set_point)<=0.9*(360/MOTOR_STEPS/Micro_step)))
  {
    output = PID_contorller.update(feedback);  /* Stepper motor should run faster if this value become lager */
    arrive = 0;
  }else
  {
    stepper.stop();
    arrive = 1;
  }
  Serial.print("output: ");
  Serial.print(output, DEC);
  Serial.print("  ");
	Serial.print("rpm: ");
	Serial.print(stepper.getRPM(), DEC);
	Serial.println("  ");
    if(output > 0 && arrive == 0)
    {
      //stepper.setRPM(map(output, 0, 180, 0, 100));
      stepper.setRPM(output);
      stepper.move(1);
    }else if(output < 0 && arrive == 0)
    {
      output *= -1;
      //stepper.setRPM(map(output, 0, 180, 0, 100));
      stepper.setRPM(output);
      stepper.move(-1);
    }
}
