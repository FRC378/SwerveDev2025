// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveStop.h"
#include "Robot.h"

CmdDriveStop::CmdDriveStop() 
{

}

void CmdDriveStop::Initialize() 
{
  robotcontainer.m_drivetrain.Stop();
  std::cout<< "CmdDriveStop" << std::endl;
}
