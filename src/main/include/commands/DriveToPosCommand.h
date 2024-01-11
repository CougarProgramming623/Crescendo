#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Timer.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>


using namespace pathplanner;

class DriveToPosCommand : public frc2::CommandHelper<frc2::Command, DriveToPosCommand> {
 public:
  explicit DriveToPosCommand();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  frc::Timer m_Timer {};
  PathPlannerTrajectory m_Trajectory;
  frc::Pose2d m_Start;
  frc::Pose2d m_End;

  double vertical;
  double horizontal;
};