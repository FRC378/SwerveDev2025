// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <iostream>

#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <frc/AnalogEncoder.h>
#include <frc/AnalogInput.h>


class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(  int driveCanID, int turnCanID, int encoderID, double absEncOffset, std::string moduleID );

  void Periodic() override;


  //Manual Control for testing
  void SetDriveMotorPower( double power);
  void SetTurnMotorPower( double power);  
  void AlignTurnEncoderToAbsouteEncoder(void);


  //Swerve State
  void SetDesiredState(frc::SwerveModuleState& state);
  
  frc::SwerveModulePosition GetPosition() const;
  

  //Absolute Turn Encoder
  double GetTurnEncoderAbsolutePosition(void);
  double GetTurnEncoderAbsolutePositionRaw(void);

  //Turn motor
  void   ResetTurnEncoder(void);
  double GetTurnEncoderPosition(void);
  void   SetTurnAngle( double angle );

  //Drive motor
  void   ResetDriveEncoder(void);
  double GetDriveEncoderPosition(void);
  void   SetDriveVelocity( double speed ); //in ft/sec
  double GetDriveVelocity( void );

 private:

  rev::spark::SparkMax                  m_driveMotor;
  rev::spark::SparkRelativeEncoder      m_driveEncoder    = m_driveMotor.GetEncoder();
  rev::spark::SparkClosedLoopController m_drivePID        = m_driveMotor.GetClosedLoopController();

  rev::spark::SparkMax                  m_turnMotor;
  rev::spark::SparkRelativeEncoder      m_turnEncoder     = m_turnMotor.GetEncoder();
  rev::spark::SparkClosedLoopController m_turnPID         = m_turnMotor.GetClosedLoopController();


  frc::AnalogEncoder m_analogEncoder;

  double m_absEncOffset;  //Offset of Absolute encoder to actual 0degrees


  //For Debugging
  std::string m_moduleID;

};
