// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>


RobotContainer robotcontainer;  //Global Variable Object

//Prototypes
void WriteToSmartDashboard(void);


Robot::Robot() 
{
  //*************************** INIT ******************************
  cout<<"RobotInit"<<endl;
  cout<<"FRC2025Beta: SwerveDriveDev"<<endl;
  cout<<"Version: " << __DATE__ <<"  "<<__TIME__<<endl<<endl; 



  //Robot Initialization
  robotcontainer.m_drivetrain.ResetDriveEncoders();
  robotcontainer.m_drivetrain.ResetTurnEncoders();
  robotcontainer.m_drivetrain.ResetOdometry();


}

void Robot::RobotPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();

  WriteToSmartDashboard();
}

void Robot::DisabledInit() 
{
  cout<<"DisabledInit"<<endl;
}

void Robot::DisabledPeriodic() 
{
}

void Robot::DisabledExit() 
{
  cout<<"Disabled Exit"<<endl;
}

void Robot::AutonomousInit() 
{
    cout<<"AutonomousInit"<<endl;
  m_autonomousCommand = robotcontainer.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() 
{
  cout<<"AutonomousExit"<<endl;
}

void Robot::TeleopInit() 
{
  cout<<"TeleopInit"<<endl;
  
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() 
{
  cout<<"TeleopExit"<<endl;
}


void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}
void Robot::TestPeriodic() {}
void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif



void WriteToSmartDashboard(void)
{

  //XBox Controllers
  frc::SmartDashboard::PutNumber("Xbox Left-Y",   robotcontainer.m_botDriver.GetLeftY()    ); 
  frc::SmartDashboard::PutNumber("Xbox Left-X",   robotcontainer.m_botDriver.GetLeftX()    ); 
  frc::SmartDashboard::PutNumber("Xbox Right-X",  robotcontainer.m_botDriver.GetRightX()   ); 



  //Gyro
  frc::SmartDashboard::PutBoolean("Gyro_IsConn",  robotcontainer.m_drivetrain.IsGyroConnected() );
  frc::SmartDashboard::PutNumber( "Gyro_Yaw",     robotcontainer.m_drivetrain.GetGyroYaw() ); 



  //Odometry
  frc::SmartDashboard::PutNumber( "odoX",  robotcontainer.m_drivetrain.GetOdometryX() ); 
  frc::SmartDashboard::PutNumber( "odoY",  robotcontainer.m_drivetrain.GetOdometryY() ); 
  frc::SmartDashboard::PutNumber( "odoH",  robotcontainer.m_drivetrain.GetOdometryHeading() ); 


}



