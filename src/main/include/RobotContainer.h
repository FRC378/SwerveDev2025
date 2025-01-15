// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc/Timer.h>

//Subsystems
#include "subsystems/Drivetrain.h"
#include "subsystems/Logging.h"
#include "subsystems/PhotonVision.h"

class RobotContainer {
 public:
  RobotContainer();


  //****************Controllers*******************
  frc::XboxController m_botDriver{0};

  frc::Timer m_timer;

  //****************Subsystems*******************
  Drivetrain m_drivetrain;
  Logging    m_logging;

  PhotonVision m_photonvision;



  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();


  //Bottom Driver Buttons
  frc2::JoystickButton m_botDriver_StartButton{&m_botDriver, frc::XboxController::Button::kStart };
  frc2::JoystickButton m_botDriver_YButton    {&m_botDriver, frc::XboxController::Button::kY };
  frc2::JoystickButton m_botDriver_XButton    {&m_botDriver, frc::XboxController::Button::kX };
  frc2::JoystickButton m_botDriver_BButton    {&m_botDriver, frc::XboxController::Button::kB };
  frc2::JoystickButton m_botDriver_AButton    {&m_botDriver, frc::XboxController::Button::kA };

  frc2::POVButton      m_botDriver_POVup      {&m_botDriver, 0};
  frc2::POVButton      m_botDriver_POVdown    {&m_botDriver, 180};
  frc2::POVButton      m_botDriver_POVleft    {&m_botDriver, 270};
  frc2::POVButton      m_botDriver_POVright   {&m_botDriver, 90};


};
