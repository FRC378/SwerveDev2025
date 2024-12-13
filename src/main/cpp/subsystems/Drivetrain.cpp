// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotConstants.h"

#include <networktables/StructArrayTopic.h>

//Define MAX drivetrain velocities
static constexpr units::feet_per_second_t    kMaxVelocity = units::feet_per_second_t{  3.0 };
static constexpr units::degrees_per_second_t kMaxRotation = units::degrees_per_second_t{ 125 }; 


nt::StructArrayPublisher<frc::SwerveModuleState> publisher;

Drivetrain::Drivetrain() :
        m_frontLeft { FRONTLEFT_DRIVE_CAN_ID,  FRONTLEFT_TURN_CAN_ID,  FRONTLEFT_ENCODER_ID,  FRONTLEFT_ENCODER_OFFSET,  "FL"},
        m_frontRight{ FRONTRIGHT_DRIVE_CAN_ID, FRONTRIGHT_TURN_CAN_ID, FRONTRIGHT_ENCODER_ID, FRONTRIGHT_ENCODER_OFFSET, "FR"},
        m_backLeft  { BACKLEFT_DRIVE_CAN_ID,   BACKLEFT_TURN_CAN_ID,   BACKLEFT_ENCODER_ID,   BACKLEFT_ENCODER_OFFSET,   "BL"},
        m_backRight { BACKRIGHT_DRIVE_CAN_ID,  BACKRIGHT_TURN_CAN_ID,  BACKRIGHT_ENCODER_ID,  BACKRIGHT_ENCODER_OFFSET,  "BR"}
{





  std::cout << "DriveTrain...." << std::endl;

  std::cout << "FL" << m_frontLeftLocation.X().value()  << " " << m_frontLeftLocation.Y().value()  << std::endl;
  std::cout << "FR" << m_frontRightLocation.X().value() << " " << m_frontRightLocation.Y().value() << std::endl;
  std::cout << "BL" << m_backLeftLocation.X().value()   << " " << m_backLeftLocation.Y().value()   << std::endl;
  std::cout << "BR" << m_backRightLocation.X().value()  << " " << m_backRightLocation.Y().value()  << std::endl;


  publisher = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates").Publish();

}

// This method will be called once per scheduler run
void Drivetrain::Periodic() 
{

}



void Drivetrain::Drive( double xValue, double yValue, double rValue)
{


  //Velocity control
  units::feet_per_second_t    xSpeed = xValue * kMaxVelocity;
  units::feet_per_second_t    ySpeed = yValue * kMaxVelocity;
  units::degrees_per_second_t rSpeed = rValue * kMaxRotation;

  //The singular voodoo magic call to calculate all the nasty swerve math!
  // *** NOTE *** state velocities are conveterd to Meters per Second!!!!!!
  // *** NOTE *** kinematics defines positive X-Axis as forward,  positive y-axis is left  (See kinematic instantiation in Drivetrain.h)
  auto states = m_kinematics.ToSwerveModuleStates( frc::ChassisSpeeds{xSpeed, ySpeed, rSpeed} );

  //Normalize velocities
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxVelocity);

  //Pull individual elements from array
  auto [fl, fr, bl, br] = states;

  //Set Desired States
  m_frontLeft.SetDesiredState(  fr );
  m_frontRight.SetDesiredState( fl );
  m_backLeft.SetDesiredState(   br );
  m_backRight.SetDesiredState(  bl );


    publisher.Set(
      std::vector{
        fl,
        fr,
        bl,
        br
      }
    );


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

  void Drivetrain::ForceAllTurnAngle( double angle )
  {

    frc::SwerveModuleState state = {0_mps, frc::Rotation2d{ units::degree_t{ angle} } };

    m_frontLeft.SetDesiredState(  state  ); 
    m_frontRight.SetDesiredState(  state  ); 
    m_backLeft.SetDesiredState(  state  ); 
    m_backRight.SetDesiredState(  state  ); 

  }


  //Encoders
  void Drivetrain::ResetDriveEncoders(void)
  {
    m_frontLeft.ResetDriveEncoder();
    m_frontRight.ResetDriveEncoder();
    m_backLeft.ResetDriveEncoder();
    m_backRight.ResetDriveEncoder();
  }
  void Drivetrain::ResetTurnEncoders(void)
  {
    m_frontLeft.ResetTurnEncoder();
    m_frontRight.ResetTurnEncoder();
    m_backLeft.ResetTurnEncoder();
    m_backRight.ResetTurnEncoder();
  }