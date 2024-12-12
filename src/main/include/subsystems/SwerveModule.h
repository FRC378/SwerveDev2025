// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <iostream>

#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include <frc/AnalogEncoder.h>
#include <frc/AnalogInput.h>


class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(  int driveCanID, int turnCanID, int encoderID, std::string moduleID );

  void Periodic() override;


  //Manual Control
  void SetDriveMotorPower( double power);
  void SetTurnMotorPower( double power);  


  //Absolute Turn Encoder
  double GetTurnEncoderAbsolutePosition(void);

  //Turn motor
  void   ResetTurnEncoder(void);
  double GetTurnEncoderPosition(void);

  //Drive motor
  void   ResetDriveEncoder(void);
  double GetDriveEncoderPosition(void);


 private:

  rev::spark::SparkMax             m_driveMotor;
  rev::spark::SparkRelativeEncoder m_driveEncoder    = m_driveMotor.GetEncoder();

  rev::spark::SparkMax             m_turnMotor;
  rev::spark::SparkRelativeEncoder m_turnEncoder     = m_turnMotor.GetEncoder();


  frc::AnalogEncoder m_analogEncoder;




  //For Debugging
  std::string m_moduleID;

};
