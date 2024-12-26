// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include "commands/CmdDriveToPoint.h"
#include "Robot.h"


//       
//                         ^  +x
//               front     |
//             +-------+   |
//             |       |   |
//             |   +   |   |
//             |       |   |
//             +-------+   |
//                         |
//     +y <----------------0
//
//   Coordinate System:   
//      +X = Forward
//      +Y = Left
//      +r = CCW rotation
//
//  Heading: Direction FRONT should face during maneuver
//  Speed:   % max speed (range 0 - 1.0)
//  Stop:    Stop when point reached?
//  Timeout: Seconds until timeout (0=disabled)
//
CmdDriveToPoint::CmdDriveToPoint( double x, double y, double heading, double speed, bool stop, double timeout) 
{
  m_finalX   = x;
  m_finalY   = y;
  m_finalH   = heading;

  m_speed    = speed;   //range [0:1.0] - Percent of maximum drive speed
  m_stop     = stop;
  m_timeout  = timeout;

  m_closeEnough = false;

  AddRequirements( &robotcontainer.m_drivetrain );
}


void CmdDriveToPoint::Initialize() 
{

  m_closeEnough = false;

  //Start Timer if Timeout is set
  if( m_timeout > 0.0 )
  {
      m_timer.Reset();
      m_timer.Start();
  }
}


void CmdDriveToPoint::Execute()
{

  //-------------------------------------
  //  Directional Computations

  //distance
  double delta_x  = m_finalX - robotcontainer.m_drivetrain.GetOdometryX();
  double delta_y  = m_finalY - robotcontainer.m_drivetrain.GetOdometryY();
  double distance = std::hypot(delta_x, delta_y);

  //Are we close enough?
  const double CLOSE_ENOUGH = 1.0; 
  if( distance <= CLOSE_ENOUGH )
  {
    m_closeEnough = true;
  }


  //Unit vectors
  float ux = delta_x / distance;
  float uy = delta_y / distance;

  //Speed adjust
  float vx = ux * m_speed;
  float vy = uy * m_speed;

  //ToDo:  Slow down approaching coordinate.


  //-------------------------------------
  //  Rotational correction

  //Min turn power is 0.0625.
  //  Set Kp to reach 0.05 turn power at 1 deg error 
  const double TURN_MAX_VELOCITY = .75; 
  const double TURN_Kp           = (0.05 / 1.0);

  double delta_angle   = m_finalH - robotcontainer.m_drivetrain.GetGyroYaw();  //GetGyroYaw returns [-inf to +inf ]

  double vr = abs( delta_angle * TURN_Kp );

  //Limit max drive
  if( vr > TURN_MAX_VELOCITY ) vr = TURN_MAX_VELOCITY;


  //-------------------------------------
  //  Write solution to drivetrain
  //    + delta angle:  +vr to correct (CCW)
  //    - delta angle:  -vr to correct (CW)

  if( delta_angle < 0 )
    robotcontainer.m_drivetrain.Drive( vx, vy, -vr, Drivetrain::FIELDCENTRIC);
  else
    robotcontainer.m_drivetrain.Drive( vx, vy,  vr, Drivetrain::FIELDCENTRIC);

}


void CmdDriveToPoint::End(bool interrupted) 
{
  if(m_stop)
    robotcontainer.m_drivetrain.Stop();
}


bool CmdDriveToPoint::IsFinished() 
{


  //Check Distance
  if( m_closeEnough )
  {
      std::cout<<"CmdDriveToPoint: CloseEnough"<<std::endl;
      return true;
  }

  //Check timer
  if ((m_timeout>0.0) && m_timer.HasElapsed( units::second_t(m_timeout) ) )
  {
      m_timer.Stop();
      std::cout<<"CmdDriveToPoint: Timeout"<<std::endl;
      return true;
  }


  return false;
}
