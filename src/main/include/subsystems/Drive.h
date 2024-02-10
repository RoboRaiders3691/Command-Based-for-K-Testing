// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include <frc/Encoder.h>
#include <frc/RobotController.h>
#include <frc/drive/DifferentialDrive.h>
//#include "ctre/Pheonix.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "Constants.h"

class Drive : public frc2::SubsystemBase {
 public:
  Drive();

  frc2::CommandPtr ArcadeDriveCommand(std::function<double()> fwd,
                                      std::function<double()> rot);
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:

    TalonSRX m_fl{13};
    frc::Encoder m_encoder_fl{0, 1};
    TalonSRX m_fr{10};
    frc::Encoder m_encoder_fr{0, 1};
    TalonSRX m_bl{11};
    frc::Encoder m_encoder_bl{0, 1};
    TalonSRX m_br{12};
    frc::Encoder m_encoder_br{0, 1};

                              

  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            m_fl.SetVoltage(driveVoltage);
            m_fr.SetVoltage(driveVoltage);
            m_bl.SetVoltage(driveVoltage);
            m_br.SetVoltage(driveVoltage);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("drive-frontleft")
                .voltage(m_fl.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_encoder_fl.GetDistance()})
                .velocity(units::meters_per_second_t{m_encoder_fr.GetRate()});
            log->Motor("drive-frontright")
                .voltage(m_fr.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_encoder_fr.GetDistance()})
                .velocity(units::meters_per_second_t{m_encoder_fr.GetRate()});
                log->Motor("drive-backleft")
                .voltage(m_bl.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_encoder_bl.GetDistance()})
                .velocity(units::meters_per_second_t{m_encoder_br.GetRate()});
            log->Motor("drive-backright")
                .voltage(m_fr.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_encoder_br.GetDistance()})
                .velocity(units::meters_per_second_t{m_encoder_br.GetRate()});
          },
          this}};
      
};
