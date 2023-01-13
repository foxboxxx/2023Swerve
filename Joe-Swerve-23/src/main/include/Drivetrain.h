// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <numbers>

#include <frc/ADXRS450_Gyro.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, 
             units::radians_per_second_t angularVelocity,
             bool fieldRelative);

  frc::Pose2d SwerveOdometryGetPose();



  void UpdateOdometry();
  void ResetDrive();
  double GetGyro();

    static constexpr units::meters_per_second_t maxSpeed = 4.441_mps;  // 3 meters per second
    static constexpr units::radians_per_second_t maxTurnRate{2.0 * std::numbers::pi};  // 1/2 rotation per second

        // SwerveModuleState frontLeft;
        // SwerveModuleState frontRight;
        // SwerveModuleState backLeft;
        // SwerveModuleState backRight;
        // SwerveModule RFMod{1, 2, 9, true, RFZERO, false, false};
	    // SwerveModule LFMod{8, 7, 12, true, LFZERO, false, false};
	    // SwerveModule RBMod{3, 4, 10, false, RBZERO, false, false};
	    // SwerveModule LBMod{5, 6, 11, true, LBZERO, false, false};

 private:
 // **********************************************************
  frc::Translation2d frontLeftLocation{-0.3175_m,-0.3175_m};
  frc::Translation2d frontRightLocation{+0.3175_m, -0.3175_m};
  frc::Translation2d backLeftLocation{-0.3175_m, +0.3175_m};
  frc::Translation2d backRightLocation{+0.3175_m, +0.3175_m}; 

  SwerveModule RFMod{1, 2, 9, true, RFZERO, false, false};
  SwerveModule RBMod{3, 4, 10, false, RBZERO, false, false};
  SwerveModule LBMod{5, 6, 11, true, LBZERO, false, false};
  SwerveModule LFMod{8, 7, 12, true, LFZERO, false, false}; //privates since they're not referenced in another file - our code references it in robot.cpp, make this change when you get there.

//Declare Gyro
  frc::ADXRS450_Gyro gyro;

// Set up Kinematics Array
  frc::SwerveDriveKinematics<4> kinematics{
      frontLeftLocation, 
      frontRightLocation, 
      backLeftLocation,
      backRightLocation
      };

// Set up Odomotery Array
  frc::SwerveDriveOdometry<4> odometry{
      kinematics,
      gyro.GetRotation2d(),
        {LFMod.GetPosition(), 
        RFMod.GetPosition(),
        LBMod.GetPosition(), 
        RBMod.GetPosition()}
        };
};
