// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <photon/PhotonCamera.h>

class PhotonVision : public frc2::SubsystemBase {
 public:
  PhotonVision();


  int    GetTargetId(void);
  bool   IsTargetValid(void);

  double GetTargetYaw(void);
  double GetTargetDistance(void);


  void Periodic() override;

 private:

  photon::PhotonCamera camera{"photonvision"};

  int    m_targetId;  
  bool   m_targetValid;
  double m_targetYaw;
  double m_targetDistance;


};

