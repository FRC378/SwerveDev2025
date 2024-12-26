// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveTurnToOffsetAngle.h"
#include "Robot.h"

CmdDriveTurnToOffsetAngle::CmdDriveTurnToOffsetAngle( double angle, double power )
{
  m_power = power;
  m_angle = angle;

  AddRequirements({ &robotcontainer.m_drivetrain });
}


void CmdDriveTurnToOffsetAngle::Initialize() 
{
  m_calcAngle = robotcontainer.m_drivetrain.GetGyroYaw() + m_angle;
}


void CmdDriveTurnToOffsetAngle::Execute() 
{

  const double MAX_POWER = m_power;   // % of kMaxRotation in Drivetrain
  const double MIN_POWER = 0.5;       // % of kMaxRotation in Drivetrain
  const double TURN_Kp   = 0.1;

  double delta_angle = m_calcAngle - robotcontainer.m_drivetrain.GetGyroYaw();

  double turn_power = abs( delta_angle * TURN_Kp ) + MIN_POWER;

  if( turn_power > MAX_POWER ) turn_power = MAX_POWER;
  if( turn_power < MIN_POWER ) turn_power = MIN_POWER;

  if( delta_angle < 0)
    robotcontainer.m_drivetrain.Drive(0,0,  -turn_power, Drivetrain::ROBOTCENTRIC);
  else
    robotcontainer.m_drivetrain.Drive(0,0,   turn_power, Drivetrain::ROBOTCENTRIC);
}

void CmdDriveTurnToOffsetAngle::End(bool interrupted) 
{
  robotcontainer.m_drivetrain.Stop();
}

// Returns true when the command should end.
bool CmdDriveTurnToOffsetAngle::IsFinished() 
{
  double delta_angle = m_calcAngle - robotcontainer.m_drivetrain.GetGyroYaw();

  if(  abs(delta_angle) < 1.5 )
    return true;

  return false;
}
