#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>


class DustpanDown : public frc2::CommandHelper<frc2::Command, DustpanDown> {
 public:
  explicit DustpanDown();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  int id;
  double dis;
};