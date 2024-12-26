// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <iostream>


class CmdPrintText
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 CmdPrintText> {
 public:
  CmdPrintText(std::string text);

  void Initialize() override;

  private:
   std::string m_text;

};