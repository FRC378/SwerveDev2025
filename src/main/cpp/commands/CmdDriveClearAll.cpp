// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveClearAll.h"
#include "Robot.h"

CmdDriveClearAll::CmdDriveClearAll() {}

void CmdDriveClearAll::Initialize() 
{
  std::cout<< "CmdDriveClearAll" << std::endl;


  robotcontainer.m_drivetrain.ZeroGyro();
  robotcontainer.m_drivetrain.ResetDriveEncoders();
  robotcontainer.m_drivetrain.ResetTurnEncoders();
  robotcontainer.m_drivetrain.ResetOdometry();


  //Logging Test
  robotcontainer.m_logging.LogMessage("This is a LogMessage!");
  robotcontainer.m_logging.WriteComment("This is a write comment!");
  

}


void CmdDriveClearAll::Execute() {}


void CmdDriveClearAll::End(bool interrupted) {}


bool CmdDriveClearAll::IsFinished() 
{
  //Wait for gyro to complete calibration Here?
  return true;
}
