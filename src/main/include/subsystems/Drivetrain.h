// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "subsystems/SwerveModule.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();
  void Periodic() override;


  //Drive Controls
  void Drive( double xValue, double yValue, double rValue);

  void Stop( void );

  //Encoders
  void ResetDriveEncoders(void);
  void ResetTurnEncoders(void);



 private:

  // FL, FR, RL, RR
  SwerveModule m_RL;



};
