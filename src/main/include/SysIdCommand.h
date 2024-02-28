#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>


class SysIdCommand {
    public:
     SysIdCommand();
     frc2::CommandPtr GetAutonomousCommand();
     void ConfigureBindings();

    private:
     frc2::Trigger m_SysId;
};