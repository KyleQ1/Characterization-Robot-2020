// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/RobotController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <wpi/math>

#include "Constants.h"
#include "SysIdMechanism.h"
#include "rev/CANSparkMax.h"

/**
 * Represents a flywheel mechanism.
 */
class SimpleMotor : public SysIdMechanism {
 public:
  SimpleMotor() {
    // Set the distance per pulse for the flywheel encoders. We can simply use
    // the 1 divided by the resolution as that denotes one rotation of the
    // flywheel.
    m_encoder.SetDistancePerPulse(wpi::math::pi * 2.0 * kGearRatio / 512.0);

    m_encoder.Reset();

    m_leftGrbx.SetInverted(true);
    m_rightGrbx.SetInverted(false);
  }

  void SetPMotor(double value) override { m_group.Set(value); }

  double GetPEncDistance() override { return m_encoder.GetDistance(); }

  double GetPEncVelocity() override { return m_encoder.GetRate(); }

  void SimulationPeriodic();
  void Periodic() {
    frc::SmartDashboard::PutNumber("Flywheel Speed", m_encoder.GetRate());
  }

 private:
  static constexpr double kGearRatio = 8.0;
  double distance = 0;

  rev::CANSparkMax m_leftGrbx{Constants::SimpleMotor::kLeftPort,
                              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightGrbx{Constants::SimpleMotor::kRightPort,
                               rev::CANSparkMax::MotorType::kBrushless};

  frc::SpeedControllerGroup m_group{m_leftGrbx, m_rightGrbx};

  frc::Encoder m_encoder{Constants::SimpleMotor::kEncoderPorts[0],
                         Constants::SimpleMotor::kEncoderPorts[1]};

  // Simulation classes help us simulate our robot
  frc::sim::EncoderSim m_encoderSim{m_encoder};
  frc::LinearSystem<1, 1, 1> m_flywheelSystem =
      frc::LinearSystemId::IdentifyVelocitySystem<units::radians>(
          Constants::SimpleMotor::kV, Constants::SimpleMotor::kA);
  frc::sim::FlywheelSim m_flywheelSimulator{m_flywheelSystem,
                                            frc::DCMotor::NEO(2), 1.0 / 2.0};
};
