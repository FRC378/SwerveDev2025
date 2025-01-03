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

#define METER2INCH(x) (x*39.3701)

nt::StructArrayPublisher<frc::SwerveModuleState> publisher;

Drivetrain::Drivetrain() :
        m_frontLeft { FRONTLEFT_DRIVE_CAN_ID,  FRONTLEFT_TURN_CAN_ID,  FRONTLEFT_ENCODER_ID,  FRONTLEFT_ENCODER_OFFSET,  "FL"},
        m_frontRight{ FRONTRIGHT_DRIVE_CAN_ID, FRONTRIGHT_TURN_CAN_ID, FRONTRIGHT_ENCODER_ID, FRONTRIGHT_ENCODER_OFFSET, "FR"},
        m_backLeft  { BACKLEFT_DRIVE_CAN_ID,   BACKLEFT_TURN_CAN_ID,   BACKLEFT_ENCODER_ID,   BACKLEFT_ENCODER_OFFSET,   "BL"},
        m_backRight { BACKRIGHT_DRIVE_CAN_ID,  BACKRIGHT_TURN_CAN_ID,  BACKRIGHT_ENCODER_ID,  BACKRIGHT_ENCODER_OFFSET,  "BR"}
{

  std::cout << "DriveTrain Starting" << std::endl;
  
  publisher = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates").Publish();
  
  m_driveType = FIELDCENTRIC;


}

// This method will be called once per scheduler run
void Drivetrain::Periodic() 
{

  //Odometry
  m_odometry.Update( m_gyro.GetRotation2d()  ,
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()});


  m_field.SetRobotPose(m_odometry.GetPose());
  frc::SmartDashboard::PutData("Field", &m_field);                  


  //Drive Type
  frc::SmartDashboard::PutBoolean( "DriveType", m_driveType);

}



void Drivetrain::Drive( double xValue, double yValue, double rValue, driveType drivetype )
{

  //Velocity control
  units::feet_per_second_t    xSpeed = xValue * kMaxVelocity;
  units::feet_per_second_t    ySpeed = yValue * kMaxVelocity;
  units::degrees_per_second_t rSpeed = rValue * kMaxRotation;

  //The singular voodoo magic call to calculate all the nasty swerve math!
  // *** NOTE *** state velocities are conveterd to Meters per Second!!!!!!
  // *** NOTE *** kinematics defines positive X-Axis as forward,  positive y-axis is left  (See kinematic instantiation in Drivetrain.h)
  auto states = m_kinematics.ToSwerveModuleStates( 
                    (drivetype == ROBOTCENTRIC)
                    ? frc::ChassisSpeeds{xSpeed, ySpeed, rSpeed}                                                      //RobotCentric
                    : frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, m_gyro.GetRotation2d() )    //FieldCentric
                    );
    
  //Normalize velocities
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxVelocity);

  //Pull individual elements from array
  auto [fl, fr, bl, br] = states;

  //Set Desired States
  m_frontLeft.SetDesiredState(  fl );
  m_frontRight.SetDesiredState( fr );
  m_backLeft.SetDesiredState(   bl );
  m_backRight.SetDesiredState(  br );

  publisher.Set(std::vector{ fl,fr,bl,br});

}

void Drivetrain::DriveWithPower( double power )
{
  //DriveWithPower:
  //   Used for Velocity FeedForward Calibation
  m_frontLeft.SetDriveMotorPower(  power );
  m_frontRight.SetDriveMotorPower( power );
  m_backLeft.SetDriveMotorPower(   power );
  m_backRight.SetDriveMotorPower(  power );

  //Drive Straight ahead
  m_frontLeft.SetTurnAngle(  0 );
  m_frontRight.SetTurnAngle( 0 );
  m_backLeft.SetTurnAngle(   0 );
  m_backRight.SetTurnAngle(  0 );

}


void Drivetrain::Stop( void )
{
  m_frontLeft.SetDriveMotorPower(  0 );
  m_frontRight.SetDriveMotorPower( 0 );
  m_backLeft.SetDriveMotorPower(   0 );
  m_backRight.SetDriveMotorPower(  0 );
}

void Drivetrain::ForceAllTurnAngle( double angle )
{
  // "Sets" the rotational angle of each module when kinematics routiene runs
  frc::Rotation2d rot{ units::degree_t{ angle }};
  m_kinematics.ResetHeadings( {rot,rot,rot,rot} );

}

void Drivetrain::ForcePark( void )
{
  // "Sets" the rotational angle of each module when kinematics routiene runs
  m_kinematics.ResetHeadings( { frc::Rotation2d(  units::degree_t{ -45 } ),   //FL      
                                frc::Rotation2d(  units::degree_t{  45 } ),   //FR
                                frc::Rotation2d(  units::degree_t{  45 } ),   //BL
                                frc::Rotation2d(  units::degree_t{ -45 } )    //br
                              } 
                            );
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



// --- Gyro ---
// +CCW,  -CW
bool   Drivetrain::IsGyroConnected(void)
{
  return m_gyro.IsConnected();
}
double Drivetrain::GetGyroYaw(void)            //yaw: -inf to +inf
{
  return m_gyro.GetYaw().GetValue().value();
}
void   Drivetrain::ZeroGyro(void)
{
  m_gyro.SetYaw( units::degree_t{0} );
}


// --- Odometry ---
void Drivetrain::ResetOdometry(void)
{
  //Reset to (0,0)
  // ** This needs to be fixed.  Seems like there is some state storage in odometry, and this does not fully clear it
  m_odometry.ResetPosition (  frc::Rotation2d{},
                              {frc::SwerveModulePosition{},frc::SwerveModulePosition{},frc::SwerveModulePosition{},frc::SwerveModulePosition{} },
                              frc::Pose2d{}
                            );

}
double Drivetrain::GetOdometryX(void)
{
  return METER2INCH( m_odometry.GetPose().X().value() );
}
double Drivetrain::GetOdometryY(void)
{
  return METER2INCH( m_odometry.GetPose().Y().value() );
}
double Drivetrain::GetOdometryHeading(void)
{
  return m_odometry.GetPose().Rotation().Degrees().value();    //Outputs [-180 to +180]
}



void Drivetrain::SetDriveType( driveType type )
{
    m_driveType = type;
}
Drivetrain::driveType Drivetrain::GetDriveType( void )
{
    return m_driveType;
}







