#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>


class DustpanUp : public frc2::CommandHelper<frc2::Command, DustpanUp> {
 public:
  explicit DustpanUp();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;	
  int id;
  double dis;
};