// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveTurnToHeading.h"
#include "Robot.h"

CmdDriveTurnToHeading::CmdDriveTurnToHeading( double heading, double power ) 
{
  m_power   = power;
  m_heading = heading;

  AddRequirements({ &robotcontainer.m_drivetrain });
}


void CmdDriveTurnToHeading::Initialize() 
{

}


void CmdDriveTurnToHeading::Execute() 
{

  const double MAX_POWER = m_power;   // % of kMaxRotation in Drivetrain
  const double MIN_POWER = 0.5;       // % of kMaxRotation in Drivetrain
  const double TURN_Kp   = 0.1;

  double delta_angle = m_heading - robotcontainer.m_drivetrain.GetOdometryHeading();   // [-180 to +180]

  //180 discontinuity handler
  if(  delta_angle >  180.0) delta_angle -= 360.0; 
  if(  delta_angle < -180.0) delta_angle += 360.0;



  double turn_power = abs( delta_angle * TURN_Kp ) + MIN_POWER;

  if( turn_power > MAX_POWER ) turn_power = MAX_POWER;
  if( turn_power < MIN_POWER ) turn_power = MIN_POWER;

  if( delta_angle < 0)
    robotcontainer.m_drivetrain.Drive(0,0,  -turn_power, Drivetrain::ROBOTCENTRIC);
  else
    robotcontainer.m_drivetrain.Drive(0,0,   turn_power, Drivetrain::ROBOTCENTRIC);
}

void CmdDriveTurnToHeading::End(bool interrupted) 
{
  robotcontainer.m_drivetrain.Stop();
}


bool CmdDriveTurnToHeading::IsFinished() 
{
  double delta_angle = m_heading - robotcontainer.m_drivetrain.GetOdometryHeading();

  if(  abs(delta_angle) < 1.5 )
    return true;

  return false;
}
