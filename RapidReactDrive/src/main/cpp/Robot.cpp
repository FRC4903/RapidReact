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
  //initializing drive motors, encoders, and PID controllers for fancy automatic movements (note: left motors run backwards)
  CANSparkMax Left_1{1, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Left1PID = Left_1.GetPIDController();
  SparkMaxRelativeEncoder Left1E = Left_1.GetEncoder(); 

  CANSparkMax Left_2{2, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Left2PID = Left_2.GetPIDController();
  SparkMaxRelativeEncoder Left2E = Left_2.GetEncoder(); 

  CANSparkMax Right_1{3, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Right1PID = Right_1.GetPIDController();
  SparkMaxRelativeEncoder Right1E = Right_1.GetEncoder(); 

  CANSparkMax Right_2{4, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Right2PID = Right_2.GetPIDController();
  SparkMaxRelativeEncoder Right2E = Right_2.GetEncoder(); 

  //initializing arm motors (note: 6 runs backwards)
  CANSparkMax Arm_1{5, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Arm1PID = Arm_1.GetPIDController();
  SparkMaxRelativeEncoder Arm1E = Arm_1.GetEncoder(); 

  CANSparkMax Arm_2{6, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Arm2PID = Arm_2.GetPIDController();
  SparkMaxRelativeEncoder Arm2E = Arm_2.GetEncoder(); 
 
  //initializes the intake/shoot motor on the arm
  TalonSRX intake;

  //joysticks added, first is main driver, second is co-pilot with control over arms and such
  Joystick firstStick{0};
  Joystick secondStick{1};

  //oldLeft and oldRight for coasting in drive code
  float oldLeft=0;
  float oldRight=0;

  //turnratio ramps down the turn speed in drive, more comfy for drivers
  double turnratio = 0.6;

  //PID constants for drive motors
  double driveP = 1e-2, driveI = 1e-6, driveD = 0, driveIz = 0, driveFF = 0.000015, driveMaxOutput = 0.8, driveMinOutput = -0.8; 
  //PID constants for arm motors
  double armP = 1e-2, armI = 2e-6, armD = 0, armIz = 0, armFF = 0.000015, armMaxOutput = 0.5, armMinOutput = -0.5; 

  //arm positions for automatic arm mode, in order of [intake, front shoot, vertical, back shoot], based on enabling while arm is vertical
  double larmPositions[4] = {-55.0f, -12.0f, 0.0f, 12.0f};

  //manual vs automatic mode, just in case the arms don't play nice one day we can still fully control stuff
  bool manual = false;

  //speed modifier for drive in teleop, default is 50%, slow is 25, and full is 100
  double speedmod = 0.5;


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
    Arm_1.Set(0);
    Arm_2.Set(0);

    //initializes PID controllers for drive and arm motors
    initializePID(Left1PID, "left1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Left2PID, "left2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right1PID, "right1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right2PID, "right2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);

    initializePID(Arm1PID, "arm1", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);
    initializePID(Arm2PID, "arm2", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);

    intake.Set(ControlMode::PercentOutput, 0);
  }
  //no auto yet *wink wink nudge nudge auto people*
  void autonomousInit() {}
  void autonomousPeriodic() {}

  void TeleopInit() {}

  void TeleopPeriodic() {
    //variables for joystick positions, J1 for first controller, _1 is left stick _2 is right stick, x for horizontal, y for vertical
    float J1_1x = firstStick.GetRawAxis(0);
    float J1_1y = firstStick.GetRawAxis(1);
    float J1_2x = firstStick.GetRawAxis(2);
    float J1_2y = firstStick.GetRawAxis(3);

    //sets the second arm to follow the first, ...,true) sets it to be inverted
    Arm_2.Follow(Arm_1, true);
    
    //runs driveperiodic method below
    DrivePeriodic(J1_1y, J1_2x, speedmod);

    //sets arm outputs to 0 in manual mode, used to reset them if manual gets switched off suddenly
    if (manual) {
      Arm_1.Set(0);
      Arm_2.Set(0);
    }
    //when the buttons are pressed, the manual mode is activated. sets arms to run on power levels instead of position controls as a backup
    if (secondStick.GetRawButton(8) || secondStick.GetRawButton(7)) {
      manual = true;
      float armpow = (secondStick.GetRawButton(8) - secondStick.GetRawButton(7)) * 0.2;
      
      Arm_1.Set(armpow);
      Arm_2.Set(armpow);
    } else {
      manual = false;
    }

    //automatic arm controls, sets position rather than power to save drivers some headaches
    if (!manual) {
      if (secondStick.GetRawButton(1)) {
        Arm1PID.SetReference(larmPositions[3], ControlType::kPosition);
      } else if (secondStick.GetRawButton(2)) {
        Arm1PID.SetReference(larmPositions[0], ControlType::kPosition);
      } else if (secondStick.GetRawButton(3)) {
        Arm1PID.SetReference(larmPositions[1], ControlType::kPosition);
      } else if (secondStick.GetRawButton(4)) {
        Arm1PID.SetReference(larmPositions[2], ControlType::kPosition);
      }
    } 

    //speed modifiers
    if (firstStick.GetRawButton(5)) {
      speedmod=0.25;
    } else if (firstStick.GetRawButton(6)) {
      speedmod=1; 
    } else {
      speedmod=0.5;
    }

    //runs intake motor, only half power on the way in because balls kept tearing
    intake.Set(ControlMode::PercentOutput, ((secondStick.GetRawButton(6)*0.5)-secondStick.GetRawButton(5)));
    
    //puts encoder values to the shuffleboard so we can get the arm positions (important for presetting positions)
    SmartDashboard::PutNumber("Left", Arm1E.GetPosition());
    SmartDashboard::PutNumber("Right", Arm2E.GetPosition());
  }

    //drive code, copy/pasted for the 50th time from our testing phase
    void DrivePeriodic(float y, float x, float mod) {
    //square inputs, abs() so it keeps its negative or positive sign, also a speed modifier for fast or slow movement, and a modifier on x to keep it from turning too fast
    y *= abs(y) * mod;
    x *= abs(x) * mod * turnratio;

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
  //initializePID from init, sets all the coefficients and creates a spot in smartdashboard
  void initializePID(SparkMaxPIDController controller, string name, double kP, double kI, double kD, double kIz, double kFF, double kMax, double kMin) {
    //sets coefficients
    controller.SetP(kP);
    controller.SetI(kI);
    controller.SetD(kD);
    controller.SetIZone(kIz);
    controller.SetFF(kFF);
    controller.SetOutputRange(kMin,kMax);
    //creates the wanted rotations in smartdashboard, called in the PID drive method
    SmartDashboard::PutNumber("Rotations" + name , 0);
  }
  //automatic drive code, based on position rather than power, helpful for autonomous
  void autoDrive(double l, double r) {
    //grabs value from smartdashboard for wanted rotations on each side
    double leftRot = SmartDashboard::GetNumber("Rotationsleft1", 0);
    double rightRot = SmartDashboard::GetNumber("Rotationsright1", 0);

    //sets wanted position to the set rotations on each motor
    Left1PID.SetReference(leftRot, ControlType::kPosition);
    Left2PID.SetReference(leftRot, ControlType::kPosition);
    Right1PID.SetReference(rightRot, ControlType::kPosition);
    Right2PID.SetReference(rightRot, ControlType::kPosition);

    //outputs the position of each motor for testing purposes
    SmartDashboard::PutNumber("Left1 Encoder", Left1E.GetPosition());
    SmartDashboard::PutNumber("Left2 Encoder", Left2E.GetPosition());
    SmartDashboard::PutNumber("Right1 Encoder", Right1E.GetPosition());
    SmartDashboard::PutNumber("Right2 Encoder", Right2E.GetPosition());

    //pushes new set rotations to the smartdashboard
    SmartDashboard::PutNumber("Rotationsleft1", -l);
    SmartDashboard::PutNumber("Rotationsright1", r);
  }

};
//standard code, runs robot class
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
