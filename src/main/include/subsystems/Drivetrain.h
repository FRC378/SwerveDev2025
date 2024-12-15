// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "subsystems/SwerveModule.h"
#include "RobotConstants.h"

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>


class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();
  void Periodic() override;


  //Drive Controls
  void Drive( double xValue, double yValue, double rValue);

  void Stop( void );

  void ForceAllTurnAngle( double angle );

  //Encoders
  void ResetDriveEncoders(void);
  void ResetTurnEncoders(void);

  //Odometry
  void ResetOdometry(void);
  double GetOdometryX(void);
  double GetOdometryY(void);
  double GetOdometryHeading(void);




 private:


  // Coordinates of swerve modules for kinematic and odometry calculations
  //  Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
  //  
  //    BL        FL    +Y
  //     +--------+      ^
  //     |        >>     |     
  //     |        >>    -|----> +X
  //     +--------+      |
  //    BR        FR
  //
  const frc::Translation2d m_frontLeftLocation { units::inch_t{ DRIVEBASE_HEIGHT/2.0},  units::inch_t{ DRIVEBASE_WIDTH/2.0} };
  const frc::Translation2d m_frontRightLocation{ units::inch_t{ DRIVEBASE_HEIGHT/2.0},  units::inch_t{-DRIVEBASE_WIDTH/2.0} };
  const frc::Translation2d m_backLeftLocation  { units::inch_t{-DRIVEBASE_HEIGHT/2.0},  units::inch_t{ DRIVEBASE_WIDTH/2.0} };
  const frc::Translation2d m_backRightLocation { units::inch_t{-DRIVEBASE_HEIGHT/2.0},  units::inch_t{-DRIVEBASE_WIDTH/2.0} };

  frc::SwerveDriveKinematics<4> m_kinematics{ m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};


  // FL, FR, BL, BR
  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;

  //Odometry
  frc::SwerveDriveOdometry<4> m_odometry {  m_kinematics,
                                            frc::Rotation2d{},
                                            { m_frontLeft.GetPosition(),m_frontRight.GetPosition(),m_backLeft.GetPosition(),m_backRight.GetPosition()  }
                                          };

};
