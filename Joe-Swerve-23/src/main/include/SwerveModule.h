// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <numbers>

#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
#include <Rev/CANEncoder.h>
#include <ctre/phoenix/sensors/CANCoder.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc/motorcontrol/Spark.h>

#include <units/base.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int RotatorMotorNo, int DriveMotorNo, int CANCoderId,
               bool ReverseDirection, double AbsEncoderOffsetConstant, bool DriveReverse,
               bool TurnReverse);

  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;

  void SetToDesired(const frc::SwerveModuleState& state);
  double GetDrivePower();
  double GetRotatorPower();
  void ResetEncoder();
  double GetCurrentAngle();
  double GetAbsEncoderAngle();

 private:
//   static constexpr double kWheelRadius = 0.0508;
//   static constexpr int kEncoderResolution = 4096;

//   static constexpr auto kModuleMaxAngularVelocity =
//       std::numbers::pi * 1_rad_per_s;  // radians per second
//   static constexpr auto kModuleMaxAngularAcceleration =
//       std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

//   frc::PWMSparkMax m_driveMotor;
//   frc::PWMSparkMax m_turningMotor;

//   frc::Encoder m_driveEncoder;
//   frc::Encoder m_turningEncoder;

// Motor Controllers & Encoders
  rev::CANSparkMax DriveMotor;
  rev::CANSparkMax RotatorMotor;
  rev::SparkMaxRelativeEncoder * driveEncoder;
  rev::SparkMaxRelativeEncoder * turningEncoder;
  ctre::phoenix::sensors::CANCoder absEncoder;

// Absolute Encoder Val. + Offsets
  double absSignum;
  double absEncoderOffset;
  double turningEncoderOffset;

// PID Controllers:
  frc2::PIDController drivePIDController{KDP, KDI, KDD};
  //frc::ProfiledPIDController<units::radians> turningPIDController{KRP, KRI, KRD};
  frc2::PIDController turningPIDController{KRP, KRI, KRD};

//   frc2::PIDController drivePIDController{1.0, 0, 0};
//   frc::ProfiledPIDController<units::radians> m_turningPIDController{
//       1.0,
//       0.0,
//       0.0,
//       {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::length::meters> m_driveFeedforward{1_V, 3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::angle::radians> m_turnFeedforward{1_V, 0.5_V / 1_rad_per_s};
};
