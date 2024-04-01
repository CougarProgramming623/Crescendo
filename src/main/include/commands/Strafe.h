#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/Timer.h>


class Strafe : public frc2::CommandHelper<frc2::Command, Strafe> {
 public:
  explicit Strafe(int direction);

  void Initialize() override;
  void Execute() override;
  double Deadfix(double in, double deadband);
  double cubicMod(double in, double cm);
  void End(bool interrupted) override;
  bool IsFinished() override;

  double m_Direction;
  frc::Timer m_Timer;
};