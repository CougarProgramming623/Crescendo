#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Timer.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>


using namespace pathplanner;

class TrajectoryCommand : public frc2::CommandHelper<frc2::CommandBase, TrajectoryCommand> {
 public:
  explicit TrajectoryCommand(PathPlannerTrajectory trajectory);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  frc::Timer m_Timer {};
  PathPlannerTrajectory m_Trajectory;
};