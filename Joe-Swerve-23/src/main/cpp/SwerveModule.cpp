// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

// #include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(int RotatorMotorNo, 
                           int DriveMotorNo, 
                           int CANCoderId, 
                           bool ReverseDirection, 
                           double AbsEncoderOffsetConstant, 
                           bool DriveReverse,
                           bool TurnReverse):
    DriveMotor(DriveMotorNo, rev::CANSparkMax::MotorType::kBrushless),
    RotatorMotor(RotatorMotorNo, rev::CANSparkMax::MotorType::kBrushless),
    absEncoder(CANCoderId)
    {
            absEncoderOffset = AbsEncoderOffsetConstant; // Assign Constant from table in initialization
            if(ReverseDirection == true) absSignum = -1.0;
            else absSignum = 1.0;

            // Sets the drive and rotator motors reverse if the module is reverse (bool)
            DriveMotor.SetInverted(DriveReverse);
            RotatorMotor.SetInverted(TurnReverse);
            
            // Creating new encoders for each module for turning/driving, getting their readings, 
            //and setting the position and velocity factors (constant file)
            driveEncoder = new rev::SparkMaxRelativeEncoder(DriveMotor.GetEncoder());
            turningEncoder = new rev::SparkMaxRelativeEncoder(RotatorMotor.GetEncoder());
            driveEncoder -> SetPositionConversionFactor(DriveEncoderPosFactor);
            turningEncoder -> SetPositionConversionFactor(turnEncoderPosFactor);
            driveEncoder -> SetVelocityConversionFactor(DriveEncoderVelocityFactor);
            turningEncoder -> SetVelocityConversionFactor(turnEncoderVelocityFactor);
            turningPIDController.EnableContinuousInput(
            -1.0 * std::numbers::pi, 1.0 * std::numbers::pi);
            SwerveModule::ResetEncoder();


    }
//   m_driveMotor(driveMotorChannel),
//   m_turningMotor(turningMotorChannel),
//   m_driveEncoder(driveEncoderChannelA, driveEncoderChannelB),
//   m_turningEncoder(turningEncoderChannelA, turningEncoderChannelB) {
//   // Set the distance per pulse for the drive encoder. We can simply use the
//   // distance traveled for one rotation of the wheel divided by the encoder
//   // resolution.
//   m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
//                                      kEncoderResolution);

//   // Set the distance (in this case, angle) per pulse for the turning encoder.
//   // This is the the angle through an entire rotation (2 * std::numbers::pi)
//   // divided by the encoder resolution.
//   m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi /
//                                        kEncoderResolution);

//   // Limit the PID Controller's input range between -pi and pi and set the input
//   // to be continuous.
//   m_turningPIDController.EnableContinuousInput(
//       -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
// }

// Get State Method for Odometry
frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{driveEncoder -> GetVelocity()},
          //frc::Rotation2d()
          units::radian_t{turningEncoder -> GetPosition() - turningEncoderOffset}
          };
}

// Get Position Method for Odometry
frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{driveEncoder -> GetPosition()},
          units::radian_t{turningEncoder -> GetPosition() - turningEncoderOffset}};
}

// Get the wheel angle from the turning encoder and then subtract the absolute offset of said encoder
double SwerveModule::GetCurrentAngle(){
    return turningEncoder -> GetPosition() - turningEncoderOffset;
}

// Returns the angles for the designated absolute encoder
double SwerveModule::GetAbsEncoderAngle(){
    double angle = absEncoder.GetAbsolutePosition();
    angle *= (std::numbers::pi / 180.0);
    angle -= absEncoderOffset;
    // Pi Check:
    if(angle < -std::numbers::pi) angle += (2.0 * std::numbers::pi);
    else if(angle > std::numbers::pi) angle -= (2.0 * std::numbers::pi);

    return angle;
}

// Sets the encoder offset to the last recorded state and position to 0.0 for driving and turning encoder
// Doing so allows the swerve to readjust its alignment no matter where the encoder is reset
void SwerveModule::ResetEncoder(){
    turningEncoderOffset = SwerveModule::GetAbsEncoderAngle();
    driveEncoder -> SetPosition(0.0);
    turningEncoder -> SetPosition(0.0);

}

// Gets the drive power for the drive module
double SwerveModule::GetDrivePower(){
	return (driveEncoder->GetVelocity());
}

// Gets the rotator power from the rotator module
double SwerveModule::GetRotatorPower(){
  	return (turningEncoder->GetVelocity());
}

// Set desired state for the swerve
void SwerveModule::SetToDesired(const frc::SwerveModuleState& state) {
    if(std::abs(state.speed.value()) > 0.001/* != 0.0*/){
        frc::SwerveModuleState optimizedState = state.Optimize(state, (SwerveModule::GetCurrentAngle()*1_rad));

        double turningVal = turningPIDController.Calculate(SwerveModule::GetCurrentAngle(), 
                                                           optimizedState.angle.Radians().value());
        
        if(turningVal > 1.0) turningVal = 1.0;
        else if(turningVal < -1.0) turningVal = -1.0;


        DriveMotor.Set(drivePercentage * optimizedState.speed*(1.0/4.441_mps)); //Change to variable later
        RotatorMotor.Set(rotatePercentage * turningVal);
    }
    else{
        DriveMotor.Set(0.0);
        RotatorMotor.Set(0.0);
    }

//   // Optimize the reference state to avoid spinning further than 90 degrees
//   const auto state = frc::SwerveModuleState::Optimize(
//       referenceState, units::radian_t{m_turningEncoder.GetDistance()});

//   // Calculate the drive output from the drive PID controller.
//   const auto driveOutput = m_drivePIDController.Calculate(
//       m_driveEncoder.GetRate(), state.speed.value());

//   const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

//   // Calculate the turning motor output from the turning PID controller.
//   const auto turnOutput = m_turningPIDController.Calculate(
//       units::radian_t{m_turningEncoder.GetDistance()}, state.angle.Radians());

//   const auto turnFeedforward = m_turnFeedforward.Calculate(
//       m_turningPIDController.GetSetpoint().velocity);

//   // Set the motor outputs.
//   m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
//   m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);

}
