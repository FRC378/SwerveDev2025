// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GrpTest1.h"

#include "frc2/command/WaitCommand.h"
#include "commands/CmdPrintText.h"
#include "commands/CmdDriveClearAll.h"
#include "commands/CmdDriveStop.h"

#include "commands/CmdDriveToAbsolutePoint.h"
#include "commands/CmdDriveToRelativePoint.h"
#include "commands/CmdDriveTurnToHeading.h"
#include "commands/CmdDriveTurnToOffsetAngle.h"



GrpTest1::GrpTest1() 
{
  AddCommands
  (

    CmdPrintText("GrpTest1 Start"),
    CmdDriveClearAll(),

    frc2::WaitCommand(0.25_s),


/*
    CmdDriveTurnToHeading(90, 0.8),
    frc2::WaitCommand(0.1_s),

    CmdDriveTurnToHeading(180, 0.8),
    frc2::WaitCommand(0.1_s),

    CmdDriveTurnToHeading(190, 0.8),
    frc2::WaitCommand(0.1_s),

    CmdDriveTurnToHeading(-45, 0.8),
    frc2::WaitCommand(0.1_s),

    CmdDriveTurnToHeading(0, 0.8),
    frc2::WaitCommand(0.1_s),

*/

/*
  CmdDriveTurnToOffsetAngle( 45, 0.7),
  frc2::WaitCommand(0.1_s),

  CmdDriveTurnToOffsetAngle( 45, 0.7),
  frc2::WaitCommand(0.1_s),

  CmdDriveTurnToOffsetAngle( 45, 0.7),
  frc2::WaitCommand(0.1_s),


  CmdDriveTurnToOffsetAngle( -45, 0.7),
  frc2::WaitCommand(0.1_s),

  CmdDriveTurnToOffsetAngle( -45, 0.7),
  frc2::WaitCommand(0.1_s),

  CmdDriveTurnToOffsetAngle( -45, 0.7),
  frc2::WaitCommand(0.1_s),

  CmdDriveTurnToOffsetAngle( -45, 0.7),
  frc2::WaitCommand(0.1_s),

   CmdDriveTurnToOffsetAngle( 45, 0.7),
  frc2::WaitCommand(0.1_s),

  */

    CmdPrintText("Starting DriveToAbsolutePoint test"),
    CmdDriveToAbsolutePoint( 30, 0,  0, 0.99, false, 0),
    CmdDriveToAbsolutePoint( 30, 30, 0, 0.99, false, 0),
    CmdDriveToAbsolutePoint(  0, 30, 0, 0.99, false, 0),
    CmdDriveToAbsolutePoint(  0, 0,  0, 0.99, true,  0),
/*
    CmdDriveToAbsolutePoint( -40,  0,  -90,  0.5, false, 0),
    CmdDriveToAbsolutePoint( -40, -20, -180, 0.5, false, 0),
    CmdDriveToAbsolutePoint(  0,  -20, -270, 0.5, false, 0),
    CmdDriveToAbsolutePoint(  0,   0,  -180, 0.5, true,  0),

    CmdPrintText("Starting DriveToRelativePoint test"),
    CmdDriveToRelativePoint(  20,   0,  0, 0.5, false, 0),
    CmdDriveToRelativePoint(   0,  20,  0, 0.5, false, 0),
    CmdDriveToRelativePoint( -20,   0,  0, 0.5, false, 0),
    CmdDriveToRelativePoint(   0, -20,  0, 0.5, false, 0),
*/


    CmdPrintText("GrpTest1 Finished"),
    CmdDriveStop()
  );
}
