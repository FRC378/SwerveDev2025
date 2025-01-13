// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PhotonVision.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <photon/PhotonUtils.h>
#include <units/angle.h>
#include <units/length.h>

PhotonVision::PhotonVision()
{
    //m_targetId = 0;
    //frc::SmartDashboard::PutNumber("PV-MT Id",    m_targetId );

    //
    frc::SmartDashboard::PutNumber("PV_CAMERA_HEIGHT",    0 );
    frc::SmartDashboard::PutNumber("PV_TARGET_HEIGHT",    0 );
    frc::SmartDashboard::PutNumber("PV_CAMERA_PITCH",     0 );

}

// This method will be called once per scheduler run
void PhotonVision::Periodic() 
{


    //Assume the worst
    m_targetId       = 0;
    m_targetValid    = false;
    m_targetYaw      = 0.0;
    m_targetDistance = 0.0;

    // Get a list of the previous pipeline results.
    auto results = camera.GetAllUnreadResults();

    //Are there any results in the list?
    if( results.size() > 0 )
    {

        //Get the last pipeline result from the list
        photon::PhotonPipelineResult result = results[ results.size()-1];

        //Was an AprilTag detected?
        if( result.HasTargets() )
        {
            m_targetValid = true;
            
            //---- Best Target --------------------------
            photon::PhotonTrackedTarget bestTarget = result.GetBestTarget();

            m_targetId  = bestTarget.GetFiducialId();
            m_targetYaw = bestTarget.GetYaw();


            // const units::meter_t  CAMERA_HEIGHT = 8.2_in;
            // const units::meter_t  TARGET_HEIGHT = 57_in;
            // const units::radian_t CAMERA_PITCH  = 36.7_deg;

            units::meter_t   CAMERA_HEIGHT = units::inch_t{   frc::SmartDashboard::GetNumber("PV_CAMERA_HEIGHT",    0 ) };
            units::meter_t   TARGET_HEIGHT = units::inch_t{   frc::SmartDashboard::GetNumber("PV_TARGET_HEIGHT",    0 ) };
            units::radian_t  CAMERA_PITCH  = units::degree_t{ frc::SmartDashboard::GetNumber("PV_CAMERA_PITCH",     0 ) };


            //This function may calculate the floor X-Y distance rather than the hypotenuse distance
            //from camera to AprilTag
            units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                                        CAMERA_HEIGHT,
                                        TARGET_HEIGHT,
                                        CAMERA_PITCH,
                                        units::degree_t{ bestTarget.GetPitch() }
                                    );

            //Convert meter back to inches
            m_targetDistance = (double)(units::inch_t)range;
        }
    }



    //Status Update
    frc::SmartDashboard::PutBoolean("PV-MT Valid",  m_targetValid);
    frc::SmartDashboard::PutBoolean("PV-MT ID",     m_targetId);
    frc::SmartDashboard::PutNumber("PV-MT Yaw",     m_targetYaw  );
    frc::SmartDashboard::PutNumber("PV-MT Range",   m_targetDistance  );
}




int   PhotonVision::GetTargetId(void)
{
    return m_targetId;
}
bool  PhotonVision::IsTargetValid(void)
{
    return m_targetValid;
}

double PhotonVision::GetTargetYaw(void)
{
    if( m_targetValid)
        return m_targetYaw;
    else
        return 0.0;
}
double PhotonVision::GetTargetDistance(void)
{
    if( m_targetValid)
        return m_targetDistance;
    else
        return 0.0;
}