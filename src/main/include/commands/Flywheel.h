#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>


class Flywheel : public frc2::CommandHelper<frc2::Command, Flywheel> {
 public:
  explicit Flywheel();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
};