/*Rapid React code, drive and some auto
*/
#include <iostream>

#include <frc/Timer.h>
#include "frc/TimedRobot.h"

#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"
#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartDashboard/SendableChooser.h>

#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include "cameraserver/CameraServer.h"
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
  double driveP = 1e-2, driveI = 1e-6, driveD = 0, driveIz = 0, driveFF = 0.000015, driveMaxOutput = 0.2, driveMinOutput = -0.2;
  //PID constants for arm motors
  double armP = 1e-2, armI = 17e-7, armD =1e-6, armIz = 0, armFF = 0.000015, armMaxOutput = 0.5, armMinOutput = -0.5; 

  //arm positions for automatic arm mode, in order of [intake, front shoot, vertical, back shoot], based on enabling while arm is vertical
  double larmPositions[4] = {-55.0f, -12.0f, 0.0f, 14.0f};

  //manual vs automatic mode, just in case the arms don't play nice one day we can still fully control stuff
  bool manual = false;

  //speed modifier for drive in teleop, default is 50%, slow is 25, and full is 100
  double speedmod = 0.5;

  double tiltpow = 0;

  // init timer
  Timer *gameTimer = new Timer();

  //start positions of left and right side, set in autonomousinit
  float startl = 0;
  float startr = 0;

  int checkpoint;

  float circumfirence = 23.6;
  float gear_ratio = 10.7;
  float factorToConvertDistToRot = circumfirence / gear_ratio;

  // options for auto
  enum AutoOptions { Auto1, Auto2, Auto3, Auto4, Auto5 };
  SendableChooser<AutoOptions> m_chooser;

  bool climbmode = false;

  // adds ID to talons
  Robot():
    intake(7),
    climblen(8),
    climbtilt(9),
    tiltb(0),
    tiltf(1),
    armTop(2)
  {
    cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture();
    camera1.SetResolution(160, 120);
    cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture();
    camera2.SetResolution(160, 120);
  }

  // sets all motors to 0 output on init
  void RobotInit() {
    Left_1.Set(0);
    Left_2.Set(0);
    Right_1.Set(0);
    Right_2.Set(0);
    Arm_1.Set(0);
    Arm_2.Set(0);

    // add all the options to the shuffle board

    m_chooser.SetDefaultOption("taxi", AutoOptions::Auto3);
    m_chooser.AddOption("delay taxi", AutoOptions::Auto4);
    m_chooser.AddOption("2ball", AutoOptions::Auto1);
    m_chooser.AddOption("1ball", AutoOptions::Auto2);

    SmartDashboard::PutData(&m_chooser); // throw data at shuffle board

    // initializes PID controllers for drive and arm motors
    initializePID(Left1PID, "left1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Left2PID, "left2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right1PID, "right1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right2PID, "right2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);

    initializePID(Arm1PID, "arm1", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);
    initializePID(Arm2PID, "arm2", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);

    intake.Set(ControlMode::PercentOutput, 0);
  }
  void RobotPeriodic() {
    Arm_2.Follow(Arm_1, true);
  }
  
  void AutonomousInit() {
    MotorReset();
    PIDReset();
    // sets start positions to the encoder's position at start
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

    // current position of the motors
    float lpos = Left1E.GetPosition();
    float rpos = Right1E.GetPosition();

    //two ball auto
    if (m_Selection == AutoOptions::Auto1) {
        if (checkpoint == 0) {
            Arm1PID.SetReference(larmPositions[3], ControlType::kPosition);
            if (gameTimer -> Get() > 1.5_s && gameTimer -> Get() < 2.5_s) {
                intake.Set(ControlMode::PercentOutput, -1);
            }
            if (gameTimer -> Get() >= 2.5_s) {
                intake.Set(ControlMode::PercentOutput, 0);
                checkpoint = 1;
            }

        } else if (checkpoint == 1) {
            Arm1PID.SetReference(larmPositions[0], ControlType::kPosition);
            autoDrive(80/2.2, 80/2.2, startl, startr);
            intake.Set(ControlMode::PercentOutput, 1);

            if (gameTimer -> Get() >= 5_s) {
                intake.Set(ControlMode::PercentOutput, 0);
                checkpoint = 2;
            }
        } else if (checkpoint == 2) {
            Arm1PID.SetReference(larmPositions[3], ControlType::kPosition);
            autoDrive(-5/2.2, -5/2.2, startl, startr);
            
            if (gameTimer -> Get() > 8_s && gameTimer -> Get() < 10_s) {
                intake.Set(ControlMode::PercentOutput, -1);
            }

            if (gameTimer -> Get() >= 10_s) {
                intake.Set(ControlMode::PercentOutput, 0);
                Arm1PID.SetReference(larmPositions[2], ControlType::kPosition);
                checkpoint = 3;
            }
        } else if (checkpoint == 3) {
              autoDrive(100/2.2, 100/2.2, startl, startr);
        } else {
            MotorReset();
        }
    //one ball auto
    } else if (m_Selection == AutoOptions::Auto2) {
        if (checkpoint == 0) {
            Arm1PID.SetReference(larmPositions[1], ControlType::kPosition);
            if (gameTimer -> Get() > 1.5_s && gameTimer -> Get() < 2_s) {
                intake.Set(ControlMode::PercentOutput, -1);
            }
            if (gameTimer -> Get() >= 2_s) {
                intake.Set(ControlMode::PercentOutput, 0);
                Arm1PID.SetReference(larmPositions[2], ControlType::kPosition);
                
                autoDrive(-100/2.2, -100/2.2, startl, startr);
            }
        } else {
            MotorReset();
        }
    //taxi
    } else if (m_Selection == AutoOptions::Auto3) {
        if (checkpoint == 0) {
          autoDrive(-50/2.2, -50/2.2, startl, startr);
        }
        else {
            MotorReset();
        }
    //delayed taxi
    } else if (m_Selection == AutoOptions::Auto4) {
        if (checkpoint == 0) {
          if (gameTimer -> Get() >=  10_s) {
            autoDrive(-50/2.2, -50/2.2, startl, startr);
          }
        }
        else {
            MotorReset();
        }
    }
  }



  void TeleopInit() {}

  void TeleopPeriodic() {
    //variables for joystick positions, J1 for first controller, _1 is left stick _2 is right stick, x for horizontal, y for vertical
    float J1_1x = firstStick.GetRawAxis(0);
    float J1_1y = firstStick.GetRawAxis(1);
    float J1_2x = firstStick.GetRawAxis(2);
    float J1_2y = firstStick.GetRawAxis(3);
  
    //speed modifiers
    if (firstStick.GetRawButton(5)) {
      speedmod=0.25;
    } else if (firstStick.GetRawButton(6)) {
      speedmod=1; 
    } else {
      speedmod=0.5;
    }

    //runs driveperiodic method below
    DrivePeriodic(J1_1y, J1_2x, speedmod);
    
    climbmode = firstStick.GetRawButton(7);
    
    if (!climbmode) {
    
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


    //runs intake motor, only half power on the way in because balls kept tearing, also half power option for pilot so it can pick up two balls without ejecting it across the room
    intake.Set(ControlMode::PercentOutput, ((secondStick.GetRawButton(6)*0.5)-secondStick.GetRawButton(5)-firstStick.GetRawButton(8)*0.5));
    } else {
    Arm_1.Set(0);

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
    }
  }

    // drive code, copy/pasted for the 50th time from our testing phase
    void DrivePeriodic(float y, float x, float mod) {
    // square inputs, abs() so it keeps its negative or positive sign, also a speed modifier for fast or slow movement, and a modifier on x to keep it from turning too fast
    y *= abs(y) * mod;
    x *= abs(x) * mod * turnratio;

    // coast slash limiting min and max output (only -1 to 1 sent to motors), sets left to negative cause 180 degree rotation from right
    float LeftSpeed = max(-1.0f, min(1.0f, -1*((y-x)/25)+(oldLeft*24/25)));
    float RightSpeed = max(-1.0f, min(1.0f, ((y+x)/25)+(oldRight*24/25)));

    // sets motors to their corresponding speeds
    Left_1.Set(LeftSpeed);
    Left_2.Set(LeftSpeed);
    Right_1.Set(RightSpeed);
    Right_2.Set(RightSpeed);

    // saves old values for next calculation, allows for smooth coasts
    oldLeft = LeftSpeed;
    oldRight = RightSpeed;
  }
  // initializePID from init, sets all the coefficients and creates a spot in smartdashboard
  void initializePID(SparkMaxPIDController controller, string name, double kP, double kI, double kD, double kIz, double kFF, double kMax, double kMin) {
    //sets coefficients
    controller.SetP(kP);
    controller.SetI(kI);
    controller.SetD(kD);
    controller.SetIZone(kIz);
    controller.SetFF(kFF);
    controller.SetOutputRange(kMin,kMax);
    // creates the wanted rotations in smartdashboard, called in the PID drive method
    SmartDashboard::PutNumber("Rotations" + name , 0);
  }
  // automatic drive code, based on position rather than power, helpful for autonomous
  void autoDrive(double l, double r, double startl, double startr) {
    // pushes new set rotations to the smartdashboard
    SmartDashboard::PutNumber("Rotationsleft1", l+startl);
    SmartDashboard::PutNumber("Rotationsright1", -r+startr);
    // grabs value from smartdashboard for wanted rotations on each side
    double leftRot = SmartDashboard::GetNumber("Rotationsleft1", 0);
    double rightRot = SmartDashboard::GetNumber("Rotationsright1", 0);

    // sets wanted position to the set rotations on each motor
    Left1PID.SetReference(leftRot, ControlType::kPosition);
    Left2PID.SetReference(leftRot, ControlType::kPosition);
    Right1PID.SetReference(rightRot, ControlType::kPosition);
    Right2PID.SetReference(rightRot, ControlType::kPosition);

  }

  // PID reset
  void PIDReset() {
    initializePID(Left1PID, "left1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Left2PID, "left2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right1PID, "right1", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
    initializePID(Right2PID, "right2", driveP, driveI, driveD, driveIz, driveFF, driveMaxOutput, driveMinOutput);
  
    initializePID(Arm1PID, "arm1", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);
    initializePID(Arm2PID, "arm2", armP, armI, armD, armIz, armFF, armMaxOutput, armMinOutput);
  }

  // Motor Reset
  void MotorReset() {
    Left_1.Set(0);
    Left_2.Set(0);
    Right_1.Set(0);
    Right_2.Set(0);

    intake.Set(ControlMode::PercentOutput, 0);
  }
};

// standard code, runs robot class
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
