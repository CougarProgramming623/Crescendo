#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/Drivetrain.h"


class SysIdCommand : public frc2::CommandHelper<frc2::Command, SysIdCommand> {
    public:
     SysIdCommand();

     void Initialize() override;
     void Execute() override;
     frc2::CommandPtr GetAutonomousCommand();

    private:
     void ConfigureBindings();
     frc2::Trigger m_SysId;
};