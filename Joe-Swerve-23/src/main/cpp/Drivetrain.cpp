// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

// //Initialize with starting position and angle on field
// Drivetrain::Drivetrain(units::length::meter_t startingx, units::length::meter_t startingy, units::angle::radian_t  startingangle):

// Gets the gyro's current angle in radians (but a double in this case, note that it goes over 2pi)
double Drivetrain::GetGyro(){
  return (gyro.GetAngle()*(M_PI*180.0));

}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, 
                       bool fieldRelative) {
  auto states = kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, Drivetrain::GetGyro()*1_rad)
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kinematics.DesaturateWheelSpeeds(&states, Drivetrain::maxSpeed);

  auto [fl, fr, bl, br] = states;

  // Sets the modules to the desired states while driving
  RFMod.SetToDesired(fr);
  RBMod.SetToDesired(br);
  LBMod.SetToDesired(bl);
  LFMod.SetToDesired(fl);

} // Drive

void Drivetrain::UpdateOdometry() {
  odometry.Update(gyro.GetRotation2d(),
                    {LFMod.GetPosition(), 
                    RFMod.GetPosition(),
                    LBMod.GetPosition(), 
                    RBMod.GetPosition()});
} // Update Odometry

frc::Pose2d Drivetrain::SwerveOdometryGetPose(){
  return odometry.GetPose();

} // Swerve Get Odometry Pose

// Add odometry reset
void Drivetrain::ResetDrive(){
  gyro.Reset();
  LFMod.ResetEncoder();
  LBMod.ResetEncoder();
  RFMod.ResetEncoder();
  RBMod.ResetEncoder();

} // Reset Drive
