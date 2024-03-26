#pragma once
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>


class Shoot : public frc2::CommandHelper<frc2::Command, Shoot> {
 public:
  explicit Shoot();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
};