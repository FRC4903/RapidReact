/*FRC 2022 Rapid React
Code written by Fawn B. on Feb. 4, 2022
*/
#include "frc/TimedRobot.h"
//#include "rev/CANSparkMax.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include <frc/Joystick.h>

using namespace frc;
using namespace std;

class Robot: public TimedRobot {
  public:
  //Joystick axes
  int joystick_x = 0; //or 2 for other sticks
  int joystick_y = 1;
  int joystick_2 = 3;

  //left
  TalonSRX left_1;
  TalonSRX left_2;
  //right
  TalonSRX right_1;
  TalonSRX right_2;
  //joystick
  Joystick joysticknamegobrr {1};
  //Old speeds for speed calculation
  //abeiko - should be changed to startLeft, startRight
  float oldLeft=0;
  float oldRight=0;
  //construct
  Robot():
    left_1(1),
    left_2(2),
    right_1(3),
    right_2(4)
  {}
  void RobotInit() {
    //initializing
    left_1.Set(ControlMode::PercentOutput, 0);
    left_2.Set(ControlMode::PercentOutput, 0);
    right_1.Set(ControlMode::PercentOutput, 0);
    right_2.Set(ControlMode::PercentOutput, 0);
  }
  //the names explain idk what else you want
  void RobotPeriodic() {}

  void AutonomousInit() {}

  void AutonomousPeriodic() {}

  void TeleopInit() {}

  void TeleopPeriodic() {
    //joystick axes, y1 and x for arcade, y1 and y2 for separate joystick controls
    float y1 = joysticknamegobrr.GetRawAxis(joystick_y);
    float y2 = joysticknamegobrr.GetRawAxis(joystick_2);
    float x = joysticknamegobrr.GetRawAxis(joystick_x);
    //if the joysticks are just barely offset, it won't cause robot to spin constantly
    y1 = (abs(y1)<=0.05)? 0 : -y1;
    y2 = (abs(y2)<=0.05)? 0 : -y2;
    x = (abs(x)<=0.05)? 0 : x;
    //ARCADE DRIVE: alters speeds of motors to constrain within -1 and 1, slowly ramps speeds up and down so it's not jerky

    if (y1<0) {
      x=-x;
    }
  //Robot turning
    float LeftSpeed = max(-1.0f, min(1.0f, ((y1+x)/25)+(oldLeft*24/25)));
    float RightSpeed = max(-1.0f, min(1.0f, -1*((y1-x)/25)+(oldRight*24/25)));
    
    /*
    //SEPARATE JOYSTICK DRIVE: similar idea, now takes separate sticks instead of using x value
    float LeftSpeed = max(-1.0f, min(1.0f, (y1/25+oldLeft*24/25)));
    float RightSpeed = max(-1.0f, min(1.0f, (-y2/25+oldRight*24/25)));
    */
    //set motor output speeds yay
    left_1.Set(ControlMode::PercentOutput, LeftSpeed);
    left_2.Set(ControlMode::PercentOutput, LeftSpeed);
    right_1.Set(ControlMode::PercentOutput, RightSpeed);
    right_2.Set(ControlMode::PercentOutput, RightSpeed);
    //setting old speeds to current speed for next speed calc
    oldLeft=LeftSpeed;
    oldRight=RightSpeed;
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return StartRobot<Robot>();
}
#endif
