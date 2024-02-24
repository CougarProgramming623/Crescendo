#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "subsystems/Drivetrain.h"


class SysIdCommand {
    public:
     SysIdCommand();

     frc2::CommandPtr GetAutonomousCommand();

    private:
     void ConfigureBindings();
     frc2::Trigger m_SysId;
     DriveTrain m_DriveTrain;
};