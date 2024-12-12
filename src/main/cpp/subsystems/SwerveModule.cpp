// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>


SwerveModule::SwerveModule(  int driveCanID, int turnCanID, int encoderID, std::string moduleID ) :
                m_driveMotor   (driveCanID, rev::spark::SparkMax::MotorType::kBrushless),
                m_turnMotor    (turnCanID,  rev::spark::SparkMax::MotorType::kBrushless),
                m_analogEncoder(encoderID),
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
                .Inverted(false);


    m_driveMotor.Configure( driveMotorConfig,
                            rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                            rev::spark::SparkMax::PersistMode::kPersistParameters );


    //-- Turn Motor Configuration --
    turnMotorConfig
                .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
                .SmartCurrentLimit(50)
                .Inverted(false);

    turnMotorConfig.absoluteEncoder
                .Inverted(true);


    m_turnMotor.Configure( turnMotorConfig,
                            rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                            rev::spark::SparkMax::PersistMode::kPersistParameters );





}

void SwerveModule::Periodic() 
{



  //DebugOutput
  frc::SmartDashboard::PutNumber(m_moduleID + "-DrvEnc",    GetDriveEncoderPosition() ); 
  frc::SmartDashboard::PutNumber(m_moduleID + "-TurnEnc",   GetTurnEncoderPosition() ); 

  frc::SmartDashboard::PutNumber(m_moduleID + "-TurnAbs",   GetTurnEncoderAbsolutePosition() );  

  frc::SmartDashboard::PutNumber(m_moduleID + "-DrvPow",  m_driveMotor.Get() ); 
  frc::SmartDashboard::PutNumber(m_moduleID + "-TrnPwr",  m_turnMotor.Get()  );

}



void SwerveModule::SetDriveMotorPower( double power)
{
  m_driveMotor.Set(power);
}
void SwerveModule::SetTurnMotorPower( double power)
{
  m_turnMotor.Set(power);
} 




//---  ABSOLUTE TURN ENCODER -----
double SwerveModule::GetTurnEncoderAbsolutePosition(void)
{
  //ToDo:  Subtract off offset
  // .get returns range [0,1]
  return (360.0 * m_analogEncoder.Get()  );
}

//---  TURN MOTOR -----
void   SwerveModule::ResetTurnEncoder(void)
{
  m_turnEncoder.SetPosition(0);
}
double SwerveModule::GetTurnEncoderPosition(void)
{
  return m_turnEncoder.GetPosition();
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



