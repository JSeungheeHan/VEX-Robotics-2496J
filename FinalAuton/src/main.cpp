// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// inert                inertial      16              
// intakeL              motor         17              
// intakeR              motor         18              
// shooter              motor         2               
// top_right            motor         8               
// top_left             motor         1               
// bottom_right         motor         20              
// bottom_left          motor         11              
// RotationL            rotation      13              
// RotationR            rotation      19              
// roller               motor         9               
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
  printf("NEW RUN \n\n\n\n\n\n");
  inert.calibrate();
  while(inert.isCalibrating())
  {
    vex::task::sleep(100);
  }
  outake(100);
  vex::task::sleep(500);
  roller.spin(fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(1200);
  //Deploy
//First Goal
  moveForwardPID(580, 7);
  roller.spin(reverse);
  shooter.spin(fwd, -20, vex::velocityUnits::pct);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  turn(-137, 15);
  stoptake();
  moveForwardPID(700, 7);
  roller.spin(fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(300);
  roller.stop();
  shooter.stop();
//Second Goal
moveForwardPID(-1230, 10);
  outake(100);
  turn(-275, 15);
  roller.spin(fwd);
  moveForwardPID(732, 7);
  turn(-184, 15);
  roller.spin(reverse);
  shooter.spin(fwd, -20, vex::velocityUnits::pct);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  stoptake();
  moveForwardPID(760, 7);
  roller.spin(fwd);
  vex::task::sleep(1000);
  roller.stop();
  shooter.stop();
//Third Goal
moveForwardPID(-160, 7);
turn(-276, 15);
outake(100);
roller.spin(fwd, 100, vex::velocityUnits::pct);
  moveForwardPID(1538, 7);
  roller.spin(reverse);
  shooter.spin(fwd, -20, vex::velocityUnits::pct);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  turn(-227, 15);
  moveForwardPID(190, 5);
  stoptake();
  roller.spin(fwd);
  vex::task::sleep(1000);
  roller.stop();
  shooter.stop();
//Fourth Goal
  moveForwardPID(-578, 7);
  intake();
  turn(-362, 15);
  roller.spin(fwd, 100, vex::velocityUnits::pct);
  outake(100);
  moveForwardPID(1258, 8);
  roller.spin(reverse);
  shooter.spin(fwd, -20, vex::velocityUnits::pct);
  vex::task::sleep(600);
  roller.stop();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  turn(-270, 15);
  moveForwardPID(160, 5);
  stoptake();
  roller.spin(fwd);
  vex::task::sleep(1000);
  roller.stop();
  moveForwardPID(-175, 6);
  shooter.stop();
//Fifth Goal
  roller.spin(fwd);
  outake(100);
  turn(-360, 15);
  roller.spin(reverse);
  vex::task::sleep(600);
  roller.stop();
  stoptake();
  shooter.spin(fwd, 100, vex::velocityUnits:: pct);
  moveForward(100);
  vex::task::sleep(1240);
  turn(-325, 15);
  moveForward(42);
  outake(100);
  roller.spin(fwd);
  vex::task::sleep(2000);
  roller.stop();
  shooter.stop();
  vex::task::sleep(1000);
  moveForward(-40);
//Sixth Goal?

}
int main() {

  // inert.calibrate();
  // while(inert.isCalibrating())
  // {
  //   vex::task::sleep(100);
  // }
  // turn(-90, 100);
  // moveForwardPID(720, 1000);
  // turn(-270, 100);
  // moveForwardPID(720, 1000);
  // turn(-135, 100);
  Controller1.Screen.clearScreen();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);
  
}