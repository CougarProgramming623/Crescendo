#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>


class LockOn : public frc2::CommandHelper<frc2::Command, LockOn> {
 public:
  explicit LockOn();

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  double Deadfix(double in, double deadband);
  double cubicMod(double in, double cm);

  frc::Rotation2d m_GoalTheta;
  double m_AprilTagID;
  double m_PriorityId;
  double m_AngleError;
};