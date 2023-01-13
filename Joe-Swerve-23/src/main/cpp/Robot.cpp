// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>

#include <string>

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/kinematics/SwerveModuleState.h>

#include "Constants.h"
#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
 // Comment out if issues
   Robot():
      swerveBot()
      { 
        dash -> init();
      }

// inits might be useless and causing the issue
  void RobotInit() {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    frc::SmartDashboard::PutData("Field",&m_field);
  }

  void AutonomousInit() {
    fieldRelative = FIELD_ORIENTED; // Use Robot Oriented Control for Auto
    swerveBot.ResetDrive();
  }

  // End of comment

  void AutonomousPeriodic() override {
    ControlledDrive(false);
    swerveBot.UpdateOdometry();
  }

  void TeleopPeriodic() override { 
    // Uncomment if broken
    //m_field.SetRobotPose(swerveBot.SwerveOdometryGetPose());
    ControlledDrive(true); 
    }

 private:
  Drivetrain swerveBot; // Construct drivetrain object
  frc::XboxController controller{3}; // Xbox controller in port 3
  frc::SmartDashboard* dash; // Initialize smart dashboard

  // Possible useless code - comment out if issue persists:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::Field2d m_field;
  bool fieldRelative;
  frc::Field2d field;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> xSpeedLimiter{30 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> ySpeedLimiter{30 / 1_s}; // Used to be 3 / 1_s
  frc::SlewRateLimiter<units::scalar> rotLimiter{30 / 1_s}; // Used to be 3 / 1_s

  void ControlledDrive(bool fieldRelative) { // Method to drive robot with Xbox Controller
    const auto xSpeed = -xSpeedLimiter.Calculate(
                        frc::ApplyDeadband(controller.GetLeftY(), 0.4)) // Dead band used to be 0.02
                        * Drivetrain::maxSpeed;

    const auto ySpeed = -ySpeedLimiter.Calculate(
                        frc::ApplyDeadband(controller.GetLeftX(), 0.4)) // Dead band used to be 0.02
                        * Drivetrain::maxSpeed;

    const auto rot = -rotLimiter.Calculate(
                     frc::ApplyDeadband(controller.GetRightX(), 0.4)) // Dead band used to be 0.02
                     * Drivetrain::maxTurnRate;

    swerveBot.Drive(xSpeed, ySpeed, rot, fieldRelative);

//***swerveBot.UpdateOdometry(); //Uncomment later on to update the odometry - only needed for readings/auto not to drive
   
   
   // *IGNORE, ONLY UNCOMMENT OUT IF WE GET THE DRIVE WORKING*
    //dash->PutNumber("ABSLFPos",swerve.GetValue(L_FRONT, ABS_ANGLE));
    /*
    dash->PutNumber("ABSLBPos",swerve.LFMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSLBPos",swerve.LBMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSRFPos",swerve.RFMod.GetAbsEncoderAngle());
    dash->PutNumber("ABSRBPos",swerve.RBMod.GetAbsEncoderAngle());
    dash->PutNumber("LFPos",swerve.LFMod.GetCurrentAngle());
    dash->PutNumber("LBPos",swerve.LBMod.GetCurrentAngle());
    dash->PutNumber("RFPos",swerve.RFMod.GetCurrentAngle());
    dash->PutNumber("RBPos",swerve.RBMod.GetCurrentAngle());
    dash->PutNumber("RLFPos",swerve.LFMod.GetRotatorPower());
    dash->PutNumber("RLBPos",swerve.LBMod.GetRotatorPower());
    dash->PutNumber("RRFPos",swerve.RFMod.GetRotatorPower());
    dash->PutNumber("RRBPos",swerve.RBMod.GetRotatorPower()); 
    dash->PutNumber("DLFPos",swerve.LFMod.GetDrivePower());
    dash->PutNumber("DLBPos",swerve.LBMod.GetDrivePower());
    dash->PutNumber("DRFPos",swerve.RFMod.GetDrivePower());
    dash->PutNumber("DRBPos",swerve.RBMod.GetDrivePower());
    dash->PutNumber("Gyro", swerve.GetGyro());*/
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
