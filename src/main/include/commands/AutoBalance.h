#pragma once

#include <AHRS.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>

class AutoBalance
    : public frc2::CommandHelper<frc2::Command, AutoBalance> {
    
public:
  explicit AutoBalance();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  bool balanced;
  double m_currentAngleX;
  double m_currentAngleY;
  double m_currentAngleT;
  bool m_IsBalancing;
};