// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Logging.h"

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include "frc/DriverStation.h"
#include <iostream>



//Custom user log variables
wpi::log::StringLogEntry userComment;



Logging::Logging()
{
    m_enabled = false;
}


// This method will be called once per scheduler run
void Logging::Periodic() 
{
    if( !m_enabled) return;

}


bool Logging::GetLoggingEnable(void)
{
    return m_enabled;
}

void Logging::SetLoggingEnable(bool state)
{
    if( state )
    {
        frc::DataLogManager::Start();
        std::cout << "Logging ENABLED" << std::endl;

        //Calls to GetLog starts the Manager.  Place setup here then.
        wpi::log::DataLog& log = frc::DataLogManager::GetLog();

        //Enable DriverStation and Joystick logging
        frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());


        //Setup custom log entries
        userComment = wpi::log::StringLogEntry(log,"/user/comment");


        m_enabled = true;
    }
    else
    {
        frc::DataLogManager::Stop();      /// *** CAlling of STOP crashes program!
        //std::cout << "Logging Disabled" << std::endl;
        m_enabled = false;
    }

}

void Logging::WriteComment(std::string comment )
{
    if( m_enabled )
        userComment.Append(comment);
}
void Logging::LogMessage(std::string message )
{
    if( m_enabled )
        frc::DataLogManager::Log(message);
}