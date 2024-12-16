// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdPrintText.h"


CmdPrintText::CmdPrintText(std::string text) 
{
  m_text = text;
}

// Called when the command is initially scheduled.
void CmdPrintText::Initialize() 
{
  std::cout << m_text << std::endl;
}
