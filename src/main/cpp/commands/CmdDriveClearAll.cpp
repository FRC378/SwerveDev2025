// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveClearAll.h"
#include "Robot.h"

CmdDriveClearAll::CmdDriveClearAll() {}

void CmdDriveClearAll::Initialize() 
{
  std::cout<< "CmdDriveClearAll" << std::endl;


  robotcontainer.m_drivetrain.ResetDriveEncoders();
  robotcontainer.m_drivetrain.ResetTurnEncoders();
  
}


void CmdDriveClearAll::Execute() {}


void CmdDriveClearAll::End(bool interrupted) {}


bool CmdDriveClearAll::IsFinished() 
{
  //Wait for gyro to complete calibration Here?
  return true;
}
