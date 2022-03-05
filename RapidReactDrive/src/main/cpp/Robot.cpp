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
  //encoders for arm motors, used to find arm positions for controls later
 

  //initializes the intake/shoot motor on the arm
  TalonSRX intake;

  //first joystick added
  Joystick firstStick{0};
  Joystick secondStick{1};

  //oldLeft and oldRight for coasting in drive code
  float oldLeft=0;
  float oldRight=0;

  double driveP = 1e-2, driveI = 1e-6, driveD = 0, driveIz = 0, driveFF = 0.000015, driveMaxOutput = 0.8, driveMinOutput = -0.8; 
  
  double armP = 1e-2, armI = 1e-6, armD = 0, armIz = 0, armFF = 0.000015, armMaxOutput = 0.8, armMinOutput = -0.8; 

  double armPositions[4] = {1, 2, 3, 4};
  int curPos = 0;

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

    initializePID(Left1PID, "left1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Left2PID, "left2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right1PID, "right1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right2PID, "right2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);

    initializePID(Right2PID, "arm1", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);
    initializePID(Right2PID, "arm2", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);

    intake.Set(ControlMode::PercentOutput, 0);
  }
  void autonomousInit() {
    Left1E.SetPosition(0);
    Right1E.SetPosition(0);
    Left2E.SetPosition(0);
    Right2E.SetPosition(0);
  }
  void autonomousPeriodic() {
    autoDrive(10, 10);
  }
  void TeleopInit() {
    Left1E.SetPosition(0);
    Right1E.SetPosition(0);
    Left2E.SetPosition(0);
    Right2E.SetPosition(0);
    autoDrive(10, 10);
  }
  void TeleopPeriodic() { //NOTE: inputs likely to change this is in no way ideal
    //variables for joystick positions, J1 for first controller, _1 is left stick _2 is right stick, x for horizontal, y for vertical
    float J1_1x = firstStick.GetRawAxis(0);
    float J1_1y = firstStick.GetRawAxis(1);
    float J1_2x = firstStick.GetRawAxis(2);
    float J1_2y = firstStick.GetRawAxis(3);
    
    //runs driveperiodic method below
    DrivePeriodic(J1_1y, J1_2x, speedmod);

    if (secondStick.GetRawButton(1)) {
      curPos = 3;
    } else if (secondStick.GetRawButton(2)) {
      curPos = 1;
    } else if (secondStick.GetRawButton(3)) {
      curPos = 2;
    } else {
      curPos = 0;
    }

    if (firstStick.GetRawButton(5)) {
      speedmod=0.25;
    } else if (firstStick.GetRawButton(6)) {
      speedmod=1; 
    } else {
      speedmod=0.5;
    }

    Arm1PID.SetReference(armPositions[curPos], ControlType::kPosition);
    Arm2PID.SetReference(-armPositions[curPos], ControlType::kPosition);

    //runs intake motor, not currently hooked up but will roll wheels to pop balls in and out of the arm mechanism
    intake.Set(ControlMode::PercentOutput, (firstStick.GetRawButton(3)-firstStick.GetRawButton(1)));

    float armpow = (firstStick.GetRawButton(8)-firstStick.GetRawButton(7))*0.2;

    //not fancy joystick controls for arm motors, rotates arm2 opposite since it's rotated 180 and runs backwards NOTE: don't snap my precious baby's arms
    Arm_1.Set(armpow);
    Arm_2.Set(-1*armpow);

    //puts encoder values to the shuffleboard so we can get the arm positions (important for presetting positions)
    SmartDashboard::PutNumber("Left", Arm1E.GetPosition());
    SmartDashboard::PutNumber("Right", Arm2E.GetPosition());
  }
    //drive code, copy/pasted for the 50th time from our testing phase
    void DrivePeriodic(float y, float x, float mod) {
    //square inputs, abs() so it keeps its negative or positive sign
    y *= abs(y);
    x *= abs(x);

    y *=mod;
    x *=mod;

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
  void autoDrive(float l, float r) {
    //grabs value from smartdashboard for wanted rotations on each side
    float leftRot = SmartDashboard::GetNumber("Rotationsleft1", 0);
    float rightRot = SmartDashboard::GetNumber("Rotationsright1", 0);

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
