#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>


class Lock180 : public frc2::CommandHelper<frc2::Command, Lock180> {
 public:
  explicit Lock180();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  double Deadfix(double in, double deadband);
  double cubicMod(double in, double cm);

  frc::Rotation2d m_GoalTheta;
};