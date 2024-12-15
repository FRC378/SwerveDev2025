// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>
#include <cmath>

#include <frc/kinematics/SwerveModuleState.h>


//
//  Swerve Modules is a MK4i L2 with Billet wheels
//  www.swervedrivespecialties.com/products/mk4i-swerve-module

// MK4i has 150/7:1 turn ratio
//SparkMax reports back in rotations
//  (360 degrees * rotations) / ( 150/7:1 )   results in turn angle in degrees from SparkMax Rotations
#define TURN_ENCODER_FACTOR    ( (double)(360.0 / (150.0 / 7.0) ) )     //DEGREES per rotation

//MK4i L2 has 6.75 reduction
//  Neo Freespeed 5820 RPM
//  Tire Diameter = 4inches, perimeter = 4PI  (2*pi*r)
//  Therefore,Max free speed = 5820 /60   / 6.75   * 4PI / 12 = 15 ft/sec
// SparkMax reports back in rotations
//   Diameter * pi / 6.75   results in drive distance in inches from SparkMax Rotations 
#define DRIVE_ENCODER_FACTOR   ( (double)( 4.0 * M_PI)/ (6.75) )    //INCHES per rotation

// SparkMax reports back in rotations per minute,  12 inches per foot,  60 seconds in a minute
//  velocity = DRIVE_ENCODER_FACTOR / ( ft_per_inch * seconds_per_minute )
#define DRIVE_VELOCITY_FACTOR  ( (double)( DRIVE_ENCODER_FACTOR / (60.0 * 12.0) ) )  // ft/sec
#define DRIVE_VELOCITY_MAX     ( (double)15.0) //ft/sec


//Kinematics operates in meters_per_sec.  Convert back to feetPerSec
#define MPS2FPS(x)  (x*3.281)

//Function to bound turn encoder position to positive [0-360] degrees
double BoundDegrees(double angle)
{
  double fmodAngle = fmod(angle,360.0);
  return fmodAngle<0?(fmodAngle+360.0):fmodAngle;
}


SwerveModule::SwerveModule(  int driveCanID, int turnCanID, int encoderID, double absEncOffset, std::string moduleID ) :
                m_driveMotor   (driveCanID, rev::spark::SparkMax::MotorType::kBrushless),
                m_turnMotor    (turnCanID,  rev::spark::SparkMax::MotorType::kBrushless),
                m_analogEncoder(encoderID),
                m_absEncOffset (absEncOffset),
                m_moduleID     (moduleID)
{

  std::cout << "Starting " << moduleID << " SwerveModule" << std::endl;

  //SparkMax configurators...
  rev::spark::SparkMaxConfig driveMotorConfig;
  rev::spark::SparkMaxConfig turnMotorConfig;

  //-- Drive Motor Configuration --
  driveMotorConfig
              .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
              .SmartCurrentLimit(50)
              .Inverted(false)
              .ClosedLoopRampRate(0.3);


  driveMotorConfig.encoder
              .PositionConversionFactor( DRIVE_ENCODER_FACTOR  )
              .VelocityConversionFactor( DRIVE_VELOCITY_FACTOR );    

  driveMotorConfig.closedLoop
              .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
              .Pid(0.01, 0, 0)
              .VelocityFF(0.9/DRIVE_VELOCITY_MAX)   //********* THIS NEEDS TO BE CALIBRATED ******
              .OutputRange(-.95, 0.95);

  m_driveMotor.Configure( driveMotorConfig,
                          rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                          rev::spark::SparkMax::PersistMode::kPersistParameters );


  //-- Turn Motor Configuration --
  turnMotorConfig
              .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
              .SmartCurrentLimit(20)
              .Inverted(true) //Positive = CCW Rotation
              .ClosedLoopRampRate(0.1);

  turnMotorConfig.encoder
              .PositionConversionFactor( TURN_ENCODER_FACTOR )
              .VelocityConversionFactor( TURN_ENCODER_FACTOR / 60.0);             

  turnMotorConfig.closedLoop
              .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)     //Use internal encoder for PID
              .Pid( 0.015, 0, 0)
              .OutputRange(-0.5,0.5)
              .PositionWrappingEnabled(true)
              .PositionWrappingInputRange(0,360);


  m_turnMotor.Configure( turnMotorConfig,
                          rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                          rev::spark::SparkMax::PersistMode::kPersistParameters );



  //-- Turn Motor Absolute Encoder --
  m_analogEncoder.SetInverted(false);



}

void SwerveModule::Periodic() 
{


  //-------------
  //  DebugOutput
  //-------------


  //Turn Encoder
  frc::SmartDashboard::PutNumber(m_moduleID + "-TurnEnc",   GetTurnEncoderPosition() ); 
  frc::SmartDashboard::PutNumber(m_moduleID + "-TurnAbs",   GetTurnEncoderAbsolutePosition() );  
  frc::SmartDashboard::PutNumber(m_moduleID + "-TurnAbsRaw",GetTurnEncoderAbsolutePositionRaw() );  

  // //Drive
  // frc::SmartDashboard::PutNumber(m_moduleID + "-DrvVel",    GetDriveVelocity()  ); 
  frc::SmartDashboard::PutNumber(m_moduleID + "-DrvEnc",    GetDriveEncoderPosition() ); 


  // //Powers (when not using PID)
  // frc::SmartDashboard::PutNumber(m_moduleID + "-DrvPow",  m_driveMotor.Get() ); 
  // frc::SmartDashboard::PutNumber(m_moduleID + "-TrnPwr",  m_turnMotor.Get()  );

}



//Manual functions for testing
void SwerveModule::SetDriveMotorPower( double power)
{
  m_driveMotor.Set(power);
}
void SwerveModule::SetTurnMotorPower( double power)
{
  m_turnMotor.Set(power);
} 



//---  Swerve Modue State Control -----
void SwerveModule::SetDesiredState(frc::SwerveModuleState& state)
{
  //Get Current module angle
  frc::Rotation2d currTurnAngle{ units::degree_t{ GetTurnEncoderPosition() } };

  //Optimise turn rotation to angle based on requested angle and current angle
  state.Optimize( currTurnAngle );

  SetDriveVelocity(  MPS2FPS(state.speed() ) ) ;  //Need to convert MetersPerSec back to FeetPerSec
  SetTurnAngle( state.angle.Degrees().value() );
}

frc::SwerveModulePosition SwerveModule::GetPosition() const 
{
  return {units::meter_t{  units::inch_t{    m_driveEncoder.GetPosition() } },
          units::radian_t{ units::degree_t{  m_turnEncoder.GetPosition()  }  } };
}




//---  ABSOLUTE TURN ENCODER -----
double SwerveModule::GetTurnEncoderAbsolutePosition(void)
{
  // Offset Corrected position 
  return BoundDegrees( (360.0 * m_analogEncoder.Get()) - m_absEncOffset  );
  // Encoder.get returns range [0,1], multiply by 360 for degrees
}
double SwerveModule::GetTurnEncoderAbsolutePositionRaw(void)
{
  //Raw encoder output - used for module calibration
  return ( 360.0 * m_analogEncoder.Get() );
  // Encoder.get returns range [0,1], multiply by 360 for degrees
}

//---  TURN MOTOR -----
void   SwerveModule::ResetTurnEncoder(void)
{
  AlignTurnEncoderToAbsouteEncoder();
  //m_turnEncoder.SetPosition(0);
}
double SwerveModule::GetTurnEncoderPosition(void)
{
  return BoundDegrees( m_turnEncoder.GetPosition() );
  //return m_turnEncoder.GetPosition(); //Return +/- infinite degress
}

//Set Turn Angle
//  Angle of swerve wheel, in degrees
// Uses SparkMax internal PID
void   SwerveModule::SetTurnAngle( double angle )
{
  m_turnPID.SetReference( angle, rev::spark::SparkMax::ControlType::kPosition );
}

void SwerveModule::AlignTurnEncoderToAbsouteEncoder(void )
{
  m_turnEncoder.SetPosition(   GetTurnEncoderAbsolutePosition()   );
}



//---  DRIVE MOTOR -----

void   SwerveModule::ResetDriveEncoder(void)
{
    m_driveEncoder.SetPosition(0);
}
double SwerveModule::GetDriveEncoderPosition(void)
{
    return m_driveEncoder.GetPosition();
}
//Set Drive Velocity
//  Velocity of wheel, in ft/sec
// Uses SparkMax internal PID
void SwerveModule::SetDriveVelocity( double speed )
{
  //frc::SmartDashboard::PutNumber(m_moduleID + "-DrvRef",    speed  ); 
  m_drivePID.SetReference(speed, rev::spark::SparkMax::ControlType::kVelocity );
}
double SwerveModule::GetDriveVelocity( void )
{
  return m_driveEncoder.GetVelocity();
}

