/*beginning Rapid React code, basic drive and running motors so far
*/

#include "frc/TimedRobot.h"
#include "rev/CANSparkMax.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
//includes, just as many as possible without causing errors as usual

using namespace frc;
using namespace std;
using namespace rev;
//namespaces so I dont have to type frc:: in front of everything, personal preference on these for sure

class Robot: public TimedRobot {
  public:
  //initializing drive motors (note: left motors run backwards)
  CANSparkMax Left_1{1, CANSparkMax::MotorType::kBrushless};
  CANSparkMax Left_2{2, CANSparkMax::MotorType::kBrushless};
  CANSparkMax Right_1{3, CANSparkMax::MotorType::kBrushless};
  CANSparkMax Right_2{4, CANSparkMax::MotorType::kBrushless};
  
  //initializing arm motors (note: 6 runs backwards)
  CANSparkMax Arm_1{5, CANSparkMax::MotorType::kBrushless};
  CANSparkMax Arm_2{6, CANSparkMax::MotorType::kBrushless};

  //encoders for arm motors, used to find arm positions for controls later
  SparkMaxRelativeEncoder Arm1E = Arm_1.GetEncoder();
  SparkMaxRelativeEncoder Arm2E = Arm_2.GetEncoder();

  //initializes the intake/shoot motor on the arm
  TalonSRX intake;

  //first joystick added
  Joystick firstStick{0};

  //oldLeft and oldRight for coasting in drive code
  float oldLeft=0;
  float oldRight=0;

  //adds ID to talons
  Robot():
    intake(7)
  {}

  //sets all motors to 0 output on init
  void RobotInit() {
    Left_1.Set(0);
    Left_2.Set(0);
    Right_1.Set(0);
    Right_2.Set(0);

    intake.Set(ControlMode::PercentOutput, 0);
  }

  void TeleopPeriodic() { //NOTE: inputs likely to change this is in no way ideal
    //variables for joystick positions, J1 for first controller, _1 is left stick _2 is right stick, x for horizontal, y for vertical
    float J1_1x = firstStick.GetRawAxis(0);
    float J1_1y = firstStick.GetRawAxis(1);
    float J1_2x = firstStick.GetRawAxis(2);
    float J1_2y = firstStick.GetRawAxis(3);
    
    //runs driveperiodic method below
    DrivePeriodic(J1_1y, J1_2x);

    //runs intake motor, not currently hooked up but will roll wheels to pop balls in and out of the arm mechanism
    intake.Set(ControlMode::PercentOutput, J1_1x);

    //not fancy joystick controls for arm motors, rotates arm2 opposite since it's rotated 180 and runs backwards NOTE: don't snap my precious baby's arms
    Arm_1.Set(J1_2y);
    Arm_2.Set(-J1_2y);

    //puts encoder values to the shuffleboard so we can get the arm positions (important for presetting positions)
    SmartDashboard::PutNumber("Left", Arm1E.GetPosition());
    SmartDashboard::PutNumber("Right", Arm2E.GetPosition());
  }
    //drive code, copy/pasted for the 50th time from our testing phase
    void DrivePeriodic(float y, float x) {
    //square inputs, abs() so it keeps its negative or positive sign
    y *= abs(y);
    x *= abs(x);

    //coast slash limiting min and max output (only -1 to 1 sent to motors), sets left to negative cause 180 degree rotation from right
    float LeftSpeed = max(-1.0f, min(1.0f, -1*((y-x)/25)+(oldLeft*24/25)));
    float RightSpeed = max(-1.0f, min(1.0f, ((y+x)/25)+(oldRight*24/25)));

    //sets motors to their corresponding speeds
    Left_1.Set(LeftSpeed);
    Left_2.Set(LeftSpeed);
    Right_1.Set(RightSpeed);
    Right_2.Set(RightSpeed);

    //saves old values for next calculation, allows for smooth coasts
    oldLeft = LeftSpeed;
    oldRight = RightSpeed;
  }
};
//standard code, runs robot class
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
