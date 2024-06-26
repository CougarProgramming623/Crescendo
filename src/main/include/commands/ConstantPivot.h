#pragma once

#include <AHRS.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include <frc/AnalogInput.h>
#include <math.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "Util.h"
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/ParallelCommandGroup.h>


using namespace ctre::phoenix6;

class ConstantPivot : public frc2::CommandHelper<frc2::Command, ConstantPivot> {
    
public:
  explicit ConstantPivot();
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  // bool IsFinished() override;
  int stringpot;

private:

  
};

