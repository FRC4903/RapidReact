/*
*/
#include <frc/DigitalInput.h>
#include "frc/TimedRobot.h"
#include "rev/CANSparkMax.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include "cameraserver/CameraServer.h"
#include "frc/DriverStation.h"
#include "frc/Preferences.h"
#include <frc/liveWindow/LiveWindow.h>
#include <frc/smartDashboard/SendableChooser.h>
#include <frc/Timer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
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
  TalonSRX climbtilt;
  TalonSRX climblen;

  DigitalInput tiltb, tiltf, armTop;

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

  //tiltpower variable, used to zero out motors when running into limit switches
  double tiltpow = 0;

  //start positions for both drive motors
  double startl = Left1E.GetPosition();
  double startr= Right1E.GetPosition();

  //checkpoints to track auto
  int checkpoint = 0;

  //gear ratio * circumference, used to multiply by distance wanted
  double distToRotations =23.6*10.7;
  
  //timer for auto
  Timer *gameTimer = new Timer();

  //auto mode chooser
  enum AutoOptions { Auto1, Auto2, Auto3, Auto4, Auto5 };
  SendableChooser<AutoOptions> m_chooser;

  //adds ID to talons and switches
  Robot():
    intake(7),
    climblen(8),
    climbtilt(9),
    tiltb(0),
    tiltf(1),
    armTop(2)
  {
    //makes camera servers
    cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture();
    camera1.SetResolution(160, 120);
    cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture();
    camera2.SetResolution(160, 120);
  }

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

    //sets off intake
    intake.Set(ControlMode::PercentOutput, 0);

    //auto chooser
    m_chooser.SetDefaultOption("Auto 1", AutoOptions::Auto1);
    m_chooser.AddOption("Auto 2", AutoOptions::Auto2);
    m_chooser.AddOption("Auto 3", AutoOptions::Auto3);
    m_chooser.AddOption("Auto 4", AutoOptions::Auto4);
    m_chooser.AddOption("Auto 5", AutoOptions::Auto5);
    //pushes to shuffleboard
    SmartDashboard::PutData(&m_chooser);
  }

  void RobotPeriodic() {
    //sets the second arm to follow the first, ...,true) sets it to be inverted
    Arm_2.Follow(Arm_1, true);
  }
  
void AutonomousInit() {

  //sets start positions to the encoder's position at start
  startl = Left1E.GetPosition();
  startr = Right1E.GetPosition();

  // just make sure checkpoint is 0 at the start of each auto
  checkpoint = 0;

  // start the timer
  gameTimer -> Start();
  gameTimer -> Reset();
}

void AutonomousPeriodic() {
  // check our selected auto option
  AutoOptions m_Selection = m_chooser.GetSelected();

  //current position of the motors
  float lpos = Left1E.GetPosition();
  float rpos = Right1E.GetPosition();

  // throw encoder and checkpoint at shuffle board
  SmartDashboard::PutNumber("checkpoint ", checkpoint);
  SmartDashboard::PutNumber("startl ", startl);
  SmartDashboard::PutNumber("startr ", startr);

  // check whick auto is selected and run the corresponding function
  if (AutoOptions::Auto1 == m_Selection) {
    Autonomous1();
  } else if (AutoOptions::Auto2 == m_Selection) {
    Autonomous2();
  }
}
//2 ball auto
void Autonomous1() {
  // the error to allow when moving
  float error = 1.5f;
  //shoots back
  if (checkpoint == 0) {
    Arm1PID.SetReference(larmPositions[3], ControlType::kPosition);
    intake.Set(ControlMode::PercentOutput, -1.0f);
    if (gameTimer->Get() > 1_s) {
      checkpoint = 1;
    }
  }
  //drives back and intakes
  if (checkpoint == 1) {
    Arm1PID.SetReference(larmPositions[0], ControlType::kPosition);
    autoDrive(12*distToRotations, 12*distToRotations, startl, startr);
    intake.Set(ControlMode::PercentOutput, 0.5);
    if (positionOn(Left1E.GetPosition(), -12*distToRotations, error, startl) && positionOn(Right1E.GetPosition(), 12*distToRotations, error, startr)) {
      checkpoint = 2;
    }
  }
  //goes back to shoot
  if (checkpoint == 2) {
    Arm1PID.SetReference(larmPositions[3], ControlType::kPosition);
    autoDrive(0, 0, startl, startr);
    intake.Set(ControlMode::PercentOutput, 0);
    if (positionOn(Left1E.GetPosition(), 12*distToRotations, error, startl) && positionOn(Right1E.GetPosition(), -12*distToRotations, error, startr)) {
      checkpoint = 3;
    }
  }
  //shoots
  if (checkpoint == 3) {
    intake.Set(ControlMode::PercentOutput, -1);
    if (gameTimer->Get() > 9_s) {
      checkpoint = 4;
    }
  }
  //drives off tarmac
  if (checkpoint == 4) {
    autoDrive(12*distToRotations, 12*distToRotations, startl, startr);
    if (gameTimer->Get() > 12_s) {
      checkpoint = 5;
    }
  }
  //turns stuff off
  else {
    Left_1.Set(0);
    Left_2.Set(0);
    Right_1.Set(0);
    Right_2.Set(0);
    intake.Set(ControlMode::PercentOutput, 0);
    Arm1PID.SetReference(larmPositions[1], ControlType::kPosition);
  }
}
  //just runs it off tarmac after 10s
  void Autonomous2() {
    if (gameTimer->Get() > 10_s) {
      autoDrive(12*distToRotations, 12*distToRotations, startl, startr);
    }
  }
  //checks if the robot is at the desired point with error
  bool positionOn(double pos, double want, double err, double orig) {
    return (pos<=want+orig+err && pos >= want+orig-err);
  }

  void TeleopInit() {}

  void TeleopPeriodic() {
    //variables for joystick positions, J1 for first controller, _1 is left stick _2 is right stick, x for horizontal, y for vertical
    float J1_1x = firstStick.GetRawAxis(0);
    float J1_1y = firstStick.GetRawAxis(1);
    float J1_2x = firstStick.GetRawAxis(2);
    float J1_2y = firstStick.GetRawAxis(3);
    
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

    //runs intake motor, only half power on the way in because balls kept tearing, also half power option for pilot so it can pick up two balls without ejecting it across the room
    intake.Set(ControlMode::PercentOutput, ((secondStick.GetRawButton(6)*0.5)-secondStick.GetRawButton(5)-firstStick.GetRawButton(8)*0.5));
    //climb length and tilt mapped to copilot joysticks, stopped at limit switches
    double climblenpow = -1*secondStick.GetRawAxis(1);
    if (!armTop.Get() && climblenpow > 0) {
      climblenpow = 0;
    }
    climblen.Set(ControlMode::PercentOutput, climblenpow);
  
    tiltpow = -1*secondStick.GetRawAxis(3)/2;
    if (tiltb.Get() && tiltpow > 0) {
      tiltpow = 0;
    } else if (tiltf.Get() && tiltpow < 0) {
      tiltpow = 0;
    }
    climbtilt.Set(ControlMode::PercentOutput, tiltpow);

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
  void autoDrive(double l, double r, double startl, double startr) {
    //pushes new set rotations to the smartdashboard
    SmartDashboard::PutNumber("Rotationsleft1", -l+startl);
    SmartDashboard::PutNumber("Rotationsright1", r+startr);

    //grabs value from smartdashboard for wanted rotations on each side
    double leftRot = SmartDashboard::GetNumber("Rotationsleft1", 0);
    double rightRot = SmartDashboard::GetNumber("Rotationsright1", 0);

    //sets wanted position to the set rotations on each motor
    Left1PID.SetReference(leftRot, ControlType::kPosition);
    Left2PID.SetReference(leftRot, ControlType::kPosition);
    Right1PID.SetReference(rightRot, ControlType::kPosition);
    Right2PID.SetReference(rightRot, ControlType::kPosition);

    //outputs the position of each motor for testing purposes
    SmartDashboard::PutNumber("Left Encoder", Left1E.GetPosition());
    SmartDashboard::PutNumber("Right Encoder", Right1E.GetPosition());
  }

};
//standard code, runs robot class
#ifndef RUNNING_FRC_TESTS
int main() {
  return StartRobot<Robot>();
}
#endif
