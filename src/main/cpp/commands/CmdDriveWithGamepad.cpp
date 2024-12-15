// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveWithGamepad.h"
#include "Robot.h"
#include <frc/MathUtil.h>

CmdDriveWithGamepad::CmdDriveWithGamepad()
{

  AddRequirements( &robotcontainer.m_drivetrain );
}


void CmdDriveWithGamepad::Initialize() 
{
  std::cout<<"Starting CmdDriveWithGamepad"<<std::endl;
}


void CmdDriveWithGamepad::Execute() 
{

  const double DEADBAND  = 0.05;

  //Get Gamepad input
  // double leftY  = robotcontainer.m_botDriver.GetLeftY();
  // double leftX  = robotcontainer.m_botDriver.GetLeftX();
  // double rightX = robotcontainer.m_botDriver.GetRightX();

  double leftY  = -frc::ApplyDeadband( robotcontainer.m_botDriver.GetLeftY(),  DEADBAND, 1.0 );  //invert: Positive is forward
  double leftX  = -frc::ApplyDeadband( robotcontainer.m_botDriver.GetLeftX(),  DEADBAND, 1.0 );  //invert: Positive is left
  double rightX = -frc::ApplyDeadband( robotcontainer.m_botDriver.GetRightX(), DEADBAND, 1.0 );  //Invert: Positive is CCW



  //WPILib kinematics uses X-axis as forward, and Y-axis sideways.  Swap joystick inputs when calling Drive
  robotcontainer.m_drivetrain.Drive(leftY, leftX, rightX );

}


void CmdDriveWithGamepad::End(bool interrupted) 
{
  std::cout<<"End CmdDriveWithGamepad " << interrupted <<std::endl;
}


bool CmdDriveWithGamepad::IsFinished() 
{
  return false;
}
