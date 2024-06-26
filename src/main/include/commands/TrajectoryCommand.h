#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/AnalogInput.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <pathplanner/lib/path/PathPlannerTrajectory.h>


using namespace pathplanner;

class TrajectoryCommand : public frc2::CommandHelper<frc2::Command, TrajectoryCommand> {
 public:
  explicit TrajectoryCommand(PathPlannerTrajectory trajectory);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  // frc::Timer m_Timer;
  PathPlannerTrajectory m_Trajectory;
};