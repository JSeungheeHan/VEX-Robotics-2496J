// VEX V5 C++ Project with Competition Template
#include "vex.h"
using namespace vex;
void spinflywheel(int speed, int rotation)
{
  shooter.resetRotation();
  shooter.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
  while(shooter.rotation(vex::rotationUnits::deg) < rotation)
  {
      vex::task::sleep(20);
  }
  shooter.stop();
}
void resetRotations()
{
  top_right.resetRotation();
  top_left.resetRotation();
  bottom_right.resetRotation();
  bottom_left.resetRotation();
}
void moveForward(int speed)
{
top_left.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
bottom_left.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
top_right.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
bottom_right.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
}
void moveBackward(int speed)
{
top_left.spin(vex::directionType::fwd, -speed, vex::velocityUnits::pct);
bottom_left.spin(vex::directionType::fwd, -speed, vex::velocityUnits::pct);
top_right.spin(vex::directionType::rev, -speed, vex::velocityUnits::pct);
bottom_right.spin(vex::directionType::rev, -speed, vex::velocityUnits::pct);
}
void stopMotor()
{
  top_right.stop(vex::brakeType::coast);
  top_left.stop(vex::brakeType::coast);
  bottom_right.stop(vex::brakeType::coast);
  bottom_left.stop(vex::brakeType::coast);
}
void brakeMotor()
{
 top_right.stop(vex::brakeType::brake);
  top_left.stop(vex::brakeType::brake);
  bottom_right.stop(vex::brakeType::brake);
  bottom_left.stop(vex::brakeType::brake);
}
void intake()
{
  intakeL.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  intakeR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
}
void outake(int speed)
{
  intakeL.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
  intakeR.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
}
void stoptake()
{
  intakeL.stop();
  intakeR.stop();
}
void moveForwardPID(int distance, int time)
{
RotationL.resetPosition();
RotationR.resetPosition();
int timer = 0;
int speedLeft = 0;
int speedRight = 0;
  double iterate = 5;
  double kp = 30; //40, 30
  double ki = 0;  //4, 3
  double kd = 6;  //8, 6
  double max = 0;
  double integralLeft = 0;
  double integralRight = 0;
  double errorLeft = distance - RotationL.position(vex::rotationUnits::deg);
  double errorRight = distance - RotationR.position(vex::rotationUnits::deg);
  double prior_errorLeft = 0;
  double prior_errorRight = 0;
  double derivativeLeft = 0;
  double derivativeRight = 0;
  while(timer < time)
  {
      errorLeft = distance - RotationL.position(vex::rotationUnits::deg);
      integralLeft += (errorLeft);
      if(errorLeft < 0.4 && errorLeft > -0.4)
      {
        integralLeft = 0;
      }
      derivativeLeft = (errorLeft - prior_errorLeft);
      speedLeft = (kp * errorLeft) + (ki * integralLeft) + (kd * derivativeLeft);
      prior_errorLeft = errorLeft;
      top_left.spin(vex::directionType::fwd, speedLeft, vex::voltageUnits::mV);
      bottom_left.spin(vex::directionType::fwd, speedLeft, vex::voltageUnits::mV);
      Controller1.Screen.clearScreen();
    
      errorRight = distance - RotationR.position(vex::rotationUnits::deg);
      integralRight += (errorRight);
      if(errorRight < 0.4 && errorRight > -0.4)
      {
        integralRight = 0;
      }
      derivativeRight = (errorRight - prior_errorRight);
      speedRight = (kp * errorRight) + (ki * integralRight) + (kd * derivativeRight);
      prior_errorRight = errorRight;
      top_right.spin(vex::directionType::rev, speedRight, vex::voltageUnits::mV);
      bottom_right.spin(vex::directionType::rev, speedRight, vex::voltageUnits::mV);
      if(max < RotationL.position(vex::rotationUnits::deg))
      {
        max = RotationL.position(vex::rotationUnits::deg);
      }
      if(max < RotationR.position(vex::rotationUnits::deg))
      {
        max = RotationR.position(vex::rotationUnits::deg);
      }
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print(max);
      Controller1.Screen.setCursor(2,1);
      Controller1.Screen.print(RotationL.position(vex::rotationUnits::deg));
      Controller1.Screen.setCursor(3,1);
      Controller1.Screen.print(RotationR.position(vex::rotationUnits::deg));
      printf("Error: %f   Time: %d   Error: %f\n", errorLeft, timer, errorRight);
      timer = timer + 1;
      if(timer > time)
      {
        break;
      }
      if(errorLeft < 15 && errorLeft > -15 && errorRight < 15 && errorRight > -15)
      {
        break;
      }
      vex::task::sleep(10);
  }
  printf("\n");
  stopMotor();
}
void turn(int rot, int time)
{
  int timer = 0;
  double initialtarget = rot - inert.rotation();
  double speed;
  int iterate = 5;
  double kp = 60;//60
  double ki = 12.8;//12
  double kd = 0;//0
 double integral = 0;
  double error;
  double prior_error = 0;
  double derivative = 0;
  double max = 0;
  int inarow = 0;
  while(true)
  {
    error = rot - inert.rotation();
    if(error < 42 && error > -42) //42
    {
      integral += (error);
    }
    else
    {
      integral = 0;
    }
    if(error < 0.8 && error > -0.8) //1
    {
      integral = 0;
    }
    derivative = (error - prior_error);
    speed = (kp * error) + (ki * integral) + (kd * derivative);
    if((error < 0 && initialtarget > 0) || (error > 0 && initialtarget < 0))
    {
      if(error < -1)
      {
        speed = -1750;
      }
      if(error > 1)
      {
        speed = 1750;
      }
    }
    prior_error = error;
    top_left.spin(vex::directionType::fwd, speed, vex::voltageUnits::mV);
    bottom_left.spin(vex::directionType::fwd, speed, vex::voltageUnits::mV);
    top_right.spin(vex::directionType::fwd, speed, vex::voltageUnits::mV);
    bottom_right.spin(vex::directionType::fwd, speed, vex::voltageUnits::mV);
    if(inert.rotation() > max)
    {
      max = inert.rotation();
    }
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(inert.rotation());
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(max);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print(timer);
    printf("Error: %f   Time: %d   Speed: %f   KI: %f\n", error, timer, speed, integral);
    if(error > -3 && error < 3)
    {
        inarow++;
        if(inarow > 1)
        {
          break;
        }
    }
    else
    {
      inarow = 0;
    }
    if(timer > time)
    {
      break;
    }
    timer++;
  }
  printf("\n");
  stopMotor();
}

