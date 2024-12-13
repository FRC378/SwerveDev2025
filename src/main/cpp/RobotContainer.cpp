// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/CmdDriveWithGamepad.h"
#include "commands/CmdDriveClearAll.h"
#include "commands/CmdDriveForceTurnAngle.h"

RobotContainer::RobotContainer() 
{

  //******************** Subsystem Defaults ******************************
  m_drivetrain.SetDefaultCommand( CmdDriveWithGamepad() );


  //******************** Dashboard Buttons *******************************
  frc::SmartDashboard::PutData( "CmdDriveClearAll",  new CmdDriveClearAll() );
  frc::SmartDashboard::PutData( "Force 0",   new CmdDriveForceTurnAngle( 0.0  ) );
  frc::SmartDashboard::PutData( "Force 90",  new CmdDriveForceTurnAngle( 90.0 ) );



  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
