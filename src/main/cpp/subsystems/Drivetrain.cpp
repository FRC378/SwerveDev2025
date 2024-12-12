// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "RobotConstants.h"




Drivetrain::Drivetrain() :
        m_RL{ RL_DRIVE_CAN_ID, RL_TURN_CAN_ID, RL_ENCODER_ID, "RL"}
{


}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}



void Drivetrain::Drive( double xValue, double yValue, double rValue)
{

  //m_RL.SetTurnMotorPower( rValue);
  //m_RL.SetDriveMotorPower( yValue );

  m_RL.SetDriveVelocity(  yValue * 10.0 );

  double rlCurrAngle = m_RL.GetTurnEncoderPosition();

  double rlNewAngle = rlCurrAngle + (rValue * 120);

  m_RL.SetTurnAngle(rlNewAngle);

}




  void Drivetrain::Stop( void )
  {

  }




  //Encoders
  void Drivetrain::ResetDriveEncoders(void)
  {
    m_RL.ResetDriveEncoder();
  }
  void Drivetrain::ResetTurnEncoders(void)
  {
    m_RL.ResetTurnEncoder();
  }