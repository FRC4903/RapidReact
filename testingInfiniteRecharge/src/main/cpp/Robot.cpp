//basic code to run the infinite recharge robot

#include "frc/TimedRobot.h"
#include "rev/CANSparkMax.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include <frc/Joystick.h>
//all the includes and namespaces
using namespace frc;
using namespace std;
using namespace rev;

class Robot: public TimedRobot {
  public:
  //Joystick init, only one because I am a chad
  Joystick firstStick{0};
  
  //talon motors, names should be explanatory
  TalonSRX intake;
  TalonSRX conveyor;
  TalonSRX tilt;
  TalonSRX ClimbL;
  TalonSRX ClimbR;

  //each one of these contains a drive motor, drive PID, and drive motor encoder's initialization
  CANSparkMax Left1{1, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Left1PID = Left1.GetPIDController();
  SparkMaxRelativeEncoder Left1E = Left1.GetEncoder(); 

  CANSparkMax Left2{2, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Left2PID = Left2.GetPIDController();
  SparkMaxRelativeEncoder Left2E = Left2.GetEncoder();

  CANSparkMax Right1{3, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Right1PID = Right1.GetPIDController();
  SparkMaxRelativeEncoder Right1E = Right1.GetEncoder();

  CANSparkMax Right2{4, CANSparkMax::MotorType::kBrushless};
  SparkMaxPIDController Right2PID = Right2.GetPIDController();
  SparkMaxRelativeEncoder Right2E = Right2.GetEncoder();

  //just initializes shoot motors, since all I do is run them max speed
  CANSparkMax Shoot1{9, CANSparkMax::MotorType::kBrushless};
  CANSparkMax Shoot2{12, CANSparkMax::MotorType::kBrushless};

  //PID coefficients for auto-drive, subject to change since it's still kinda off
  double kP = 1e-2, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 0.8, kMinOutput = -0.8; 

  //encoders for tilt and conveyor, I am yet to add functionality though
  Encoder TiltE;
  Encoder ConveyE;

  //climbmode, toggles on button 1 since climb and drive are on same sticks
  bool climbMode = false;

  //start positions of left and right side, set in autonomousinit
  float startl = 0;
  float startr = 0;

  //oldleft and oldright used for teleop drive code as usual
  float oldLeft = 0;
  float oldRight = 0;

  //checkpoint, once I fix it will keep track of which stage of autonomous we are at
  int checkpoint = 0;

  //standard constructor for talons and encoders
  Robot():
    intake(5),
    conveyor(6),
    tilt(7),
    ClimbL(10),
    ClimbR(11),
    TiltE(8,9),
    ConveyE(6,7)
  {}

//mostly resets every motor as usual
void RobotInit() {
  //resets encoders
  TiltE.Reset();
  ConveyE.Reset();

  //resets drive motors
  Left1.Set(0);
  Left2.Set(0);
  Right1.Set(0);
  Right2.Set(0);

  //calls initializePID function below so I can type less
  initializePID(Left1PID, "left1", kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput);
  initializePID(Left2PID, "left2", kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput);
  initializePID(Right1PID, "right1", kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput);
  initializePID(Right2PID, "right2", kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput);
  
  //resets shoot motors
  Shoot1.Set(0);
  Shoot2.Set(0);

  //resets talon motors
  intake.Set(ControlMode::PercentOutput, 0);
  conveyor.Set(ControlMode::PercentOutput, 0);
  tilt.Set(ControlMode::PercentOutput, 0);
  ClimbL.Set(ControlMode::PercentOutput, 0);
  ClimbR.Set(ControlMode::PercentOutput, 0);

}
void RobotPeriodic() {}
void AutonomousInit() {
  //zeroes out encoders
  Left1E.SetPosition(0);
  Right1E.SetPosition(0);
  //sets start positions to the encoder's position at start
  startl = Left1E.GetPosition();
  startr = Right1E.GetPosition();
  checkpoint = 0;
  
  //autoDrive(-20,-20,startl, startr);
}
void AutonomousPeriodic() {
  //current position of the motors
  float lpos = Left1E.GetPosition();
  float rpos = Right1E.GetPosition();

  //checkpoint selector
  if (checkpoint == 0 && lpos + 10 < startl && rpos - 10 > startr) {
    checkpoint = 1;
  } else if (checkpoint == 1 && lpos - 10 > startl && rpos + 10 < startr) {
    checkpoint = 2;
  }
  //puts values to smartdashboard for testing
  SmartDashboard::PutNumber("checkpoint ", checkpoint);
  SmartDashboard::PutNumber("startl ", startl);
  SmartDashboard::PutNumber("startr ", startr);

  //calls autodrive with position and the amount of rotations it sets it relative to (starting position in this case)
  if (checkpoint == 0) {
    autoDrive(10, 10, startl, startr);
  } else if (checkpoint == 1) {
    autoDrive(-10, -10, startl, startr);
  } else if (checkpoint == 2) {
    autoDrive(0, 0, startl, startr);
  }
}
void TeleopInit() {}

void TeleopPeriodic() {
  //joysticks, barely use them
  float j1 = firstStick.GetRawAxis(1);
  float j2 = firstStick.GetRawAxis(3);

  //stops little nudges from zooming
  if (abs(j1)<=0.05f) {j1=0.0f;}
  if (abs(j2)<=0.05f) {j2=0.0f;}

  //toggles climbmode
  if (firstStick.GetRawButtonPressed(1)) {climbMode = !climbMode;}

  //climbmode sets joystick controls to climb arm output
  if (climbMode) {
    ClimbL.Set(ControlMode::PercentOutput, j1/2);
    ClimbR.Set(ControlMode::PercentOutput, -j2/2);
  //normal mode runs drive function and allows conveyor to move on button 6
  } else {
    DrivePeriodic(firstStick.GetRawAxis(1), firstStick.GetRawAxis(2));
    conveyor.Set(ControlMode::PercentOutput, firstStick.GetRawButton(6));
  }

  //sets intake on when button 5 pressed
  if (firstStick.GetRawButton(5)) {
    intake.Set(ControlMode::PercentOutput, -0.3);
  } else {
    intake.Set(ControlMode::PercentOutput, 0);
  }

  //sets shoot motors to full power when corresponding buttons pressed
  Shoot1.Set(firstStick.GetRawButton(8));
  Shoot2.Set(-(firstStick.GetRawButton(7)));
  
  //uses the weird position thing for tilt
  if (firstStick.GetPOV()==0 || firstStick.GetPOV()==45 || firstStick.GetPOV()==315) {
    tilt.Set(ControlMode::PercentOutput, 0.6f);
  } else if (firstStick.GetPOV()==180 || firstStick.GetPOV()==135 || firstStick.GetPOV()==225) {
    tilt.Set(ControlMode::PercentOutput, (-0.5f));
  } else {
    tilt.Set(ControlMode::PercentOutput, 0);
  }
}

//the one we've all been waiting for, its PID hell
void autoDrive(float l, float r, float startl, float startr) {
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
  SmartDashboard::PutNumber("Rotationsleft1", -l+startl);
  SmartDashboard::PutNumber("Rotationsright1", r+startr);
}

//standard drive code but in a separate method to clean it up
void DrivePeriodic(float y, float x) {
  //square inputs, abs() so it keeps its negative or positive sign
  y *= abs(y);
  x *= abs(x);

  //coast slash limiting min and max output
  float LeftSpeed = max(-1.0f, min(1.0f, ((y-x)/25)+(oldLeft*24/25)));
  float RightSpeed = max(-1.0f, min(1.0f, -1*((y+x)/25)+(oldRight*24/25)));

  //sets motors to their speeds
  Left1.Set(LeftSpeed);
  Left2.Set(LeftSpeed);
  Right1.Set(RightSpeed);
  Right2.Set(RightSpeed);

  //saves old values for next calculation
  oldLeft = LeftSpeed;
  oldRight = RightSpeed;
}

//this is where we initialize the PID for each motor
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

void DisabledInit() {}
void DisabledPeriodic() {}
void TestInit() {}
void TestPeriodic() {}

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return StartRobot<Robot>();
}
#endif
