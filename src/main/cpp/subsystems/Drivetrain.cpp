// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotConstants.h"


static constexpr units::feet_per_second_t    kMaxVelocity = units::feet_per_second_t{  8.0 };
static constexpr units::degrees_per_second_t kMaxRotation = units::degrees_per_second_t{ 180.0 }; 



Drivetrain::Drivetrain() :
        m_backLeft{ BACKLEFT_DRIVE_CAN_ID, BACKLEFT_TURN_CAN_ID, BACKLEFT_ENCODER_ID, "BL"}
{


}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}



void Drivetrain::Drive( double xValue, double yValue, double rValue)
{


  //Velocity control
  units::feet_per_second_t    xSpeed = xValue * kMaxVelocity;
  units::feet_per_second_t    ySpeed = yValue * kMaxVelocity;
  units::degrees_per_second_t rSpeed = rValue * kMaxRotation;

  //The singular voodoo magic call to calculate all the nasty swerve math!
  // *** NOTE ** state velocities are conveterd to Meters per Second!!!!!!
  auto states = m_kinematics.ToSwerveModuleStates( frc::ChassisSpeeds{xSpeed, ySpeed, rSpeed} );

  //Normalize velocities
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxVelocity);

  //Pull individual elements from array
  auto [fl, fr, bl, br] = states;

  //Set Desired States
  m_backLeft.SetDesiredState( bl );



/*
  //m_RL.SetTurnMotorPower( rValue);
  //m_RL.SetDriveMotorPower( yValue );

  m_backLeft.SetDriveVelocity(  yValue * 10.0 );

  double rlCurrAngle = m_backLeft.GetTurnEncoderPosition();

  double rlNewAngle = rlCurrAngle + (rValue * 120);

  m_backLeft.SetTurnAngle(rlNewAngle);
*/
}




  void Drivetrain::Stop( void )
  {

  }




  //Encoders
  void Drivetrain::ResetDriveEncoders(void)
  {
    m_backLeft.ResetDriveEncoder();
  }
  void Drivetrain::ResetTurnEncoders(void)
  {
    m_backLeft.ResetTurnEncoder();
  }