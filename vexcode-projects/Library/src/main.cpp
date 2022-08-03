
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// inert                inertial      1               
// intakeL              motor         6               
// intakeR              motor         7               
// flywheel             motor         8               
// top_right            motor         2               
// top_left             motor         3               
// bottom_right         motor         4               
// bottom_left          motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "library.h"
#include "vex.h"
using namespace vex;
vex::competition Competition;
void drivercontrol() {
  while(1==1)
    {
        if(Controller1.ButtonL1.pressing() == Controller1.ButtonUp.pressing())
        {
            intakeR.stop();
            intakeL.stop();
        }
        else
        {
            if(Controller1.ButtonL1.pressing())
            {
                intakeR.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); 
                intakeL.spin(vex::directionType::rev, 100, vex::velocityUnits::pct); 
            }
            else if(Controller1.ButtonUp.pressing())
            {
                intakeR.spin(vex::directionType::rev, 100, vex::velocityUnits::pct); 
                intakeL.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); 
            }
        }
        top_left.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()), vex::velocityUnits::pct); 
        bottom_left.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value()), vex::velocityUnits::pct); 
        top_right.spin(vex::directionType::rev, (Controller1.Axis3.value() - Controller1.Axis1.value()), vex::velocityUnits::pct);
        bottom_right.spin(vex::directionType::rev, (Controller1.Axis3.value() - Controller1.Axis1.value()), vex::velocityUnits::pct);
        //up
        if(Controller1.ButtonR1.pressing() == Controller1.ButtonL2.pressing())
        {
            roller.stop();
        }
        else if(Controller1.ButtonL2.pressing())
        {
            roller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else
        {
            roller.spin(vex::directionType::rev, 80, vex::velocityUnits::pct); 
        }
        //x
        if(Controller1.ButtonA.pressing())
        {
            shooter.spin(vex::directionType::fwd, 85, vex::velocityUnits::pct);
        }
        else
        {
            if(Controller1.ButtonR2.pressing() == Controller1.ButtonX.pressing())
            {
                shooter.stop();
            }
            else if(Controller1.ButtonR2.pressing())
            {
                shooter.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); 
            }
            else
            {
                shooter.spin(vex::directionType::rev, 100, vex::velocityUnits::pct); 
            }
        }
    }
}
        // This is the main loop for the driver control.
        // Each time through the loop you should update motor
        // movements based on input from the controller.
/*
    }
}*/
void autonomous() {
  
  inert.calibrate();
  while(inert.isCalibrating())
  {
    vex::task::sleep(100);
  }
  outake(100);
  roller.spin(fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(1200);
  //Deploy
//First Goal
  moveForwardPID(600, 10);
  roller.spin(reverse);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  turn(-137, 14);
  stoptake();
  moveForwardPID(700, 10);
  roller.spin(fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(300);
  roller.stop();
  shooter.stop();
//Second Goal
moveForwardPID(-1235, 13);
  outake(100);
  turn(-275, 14);
  roller.spin(fwd);
  moveForwardPID(740, 12);
  turn(-183, 14);
  roller.spin(reverse);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  stoptake();
  moveForwardPID(760, 12);
  roller.spin(fwd);
  vex::task::sleep(1000);
  roller.stop();
  shooter.stop();
//Third Goal
moveForwardPID(-140, 10);
turn(-276, 14);
outake(100);
roller.spin(fwd, 100, vex::velocityUnits::pct);
  moveForwardPID(1535, 14);
  roller.spin(reverse);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  turn(-225, 12);
  moveForwardPID(190, 10);
  stoptake();
  roller.spin(fwd);
  vex::task::sleep(1000);
  roller.stop();
  shooter.stop();
//Fourth Goal
moveForwardPID(-560, 10);
  roller.spin(fwd, 100, vex::velocityUnits::pct);
  turn(-364, 18);
  outake(100);
  moveForwardPID(1270, 14);
  roller.spin(reverse);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  turn(-270, 18);
  stoptake();
  roller.spin(fwd);
  vex::task::sleep(1000);
  roller.stop();
  shooter.stop();
//Fifth Goal
turn(-330, 72);
  roller.spin(reverse);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  moveForward(100);
  vex::task::sleep(3000);
  roller.spin(fwd);
  vex::task::sleep(2000);
  roller.stop();
  shooter.stop();
//Sixth Goal?

}
int main() {
  inert.calibrate();
  while(inert.isCalibrating())
  {
    vex::task::sleep(100);
  }
  turn(90, 72);
  
}