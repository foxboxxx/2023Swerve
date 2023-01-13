#pragma once

#define FIELD_ORIENTED true

//Gear and module related stuff
#define TurningMotorGearRatio 7.0/150.0 //find out
#define DrivingMotorGearRatio (1.0/6.75) //find out
#define WheelDiameterInMeters 0.1016 //find out
#define DriveEncoderPosFactor (DrivingMotorGearRatio * M_PI * WheelDiameterInMeters)
#define DriveEncoderVelocityFactor (DriveEncoderPosFactor / 60.0)
#define turnEncoderPosFactor (TurningMotorGearRatio * M_PI *2.0)
#define turnEncoderVelocityFactor (turnEncoderPosFactor / 60.0)
// #define maxSpeed 4.441 //Maximum wheel speed in meters per sec // uncomment if issues with the static drivetrain constant
// #define maxTurnRate 6.2 //In Radians per second // uncomment if issues with the static drivetrain constant

//Offsets for Absolute Encoders
#define RFZERO (1.761010-1.57079632)
#define RBZERO (3.204486-1.57079632)
#define LFZERO (5.514661-1.57079632)
#define LBZERO (5.787710-1.57079632)
#define MODULE_MAX_ANGULAR_VELOCITY (M_PI *1_rad_per_s)
#define MODULE_MAX_ANGULAR_ACCELERATION (M_PI*2_rad_per_s / 1_s)

//SwerveModule PID for Drive
#define KDP 1.0
#define KDI 0.0
#define KDD 0.0

//SweverModule PID for Rotate
#define KRP 1.0
#define KRI 0.0
#define KRD 0.0

//Drivetrain:
// Define values for GetValue Switch Statement
#define L_FRONT 0
#define R_FRONT 1
#define L_BACK 2
#define R_BACK 3
#define ABS_ANGLE 0

// MAX PERCENTAGE CONSTANTS
#define drivePercentage 0.4
#define rotatePercentage 0.7