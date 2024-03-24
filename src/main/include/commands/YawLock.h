#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <Robot.h>


class YawLock : public frc2::CommandHelper<frc2::Command, YawLock> {
 public:
  explicit YawLock();

  void Initialize() override;
  void Execute() override;
  double Deadfix(double in, double deadband);

  frc::Rotation2d m_GoalTheta;
};