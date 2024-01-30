#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include <frc/AnalogInput.h>
#include <math.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "Util.h"
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/ParallelCommandGroup.h>

using namespace ctre::phoenix6;

class DualMotorControl : public frc2::CommandHelper<frc2::Command, DualMotorControl> {
    
public:
  explicit DualMotorControl();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  // bool balanced;
  // double m_currentAngleX;
  // double m_currentAngleY;
  // double m_currentAngleT;
  // bool m_IsBalancing;
  hardware::TalonFX m_TestMotor1{TEST_MOTOR_1};
  hardware::TalonFX m_TestMotor2{TEST_MOTOR_2};
};