#include "SysIdCommand.h"

#include "Robot.h"
#include "frc2/command/Command.h"
#include "Constants.h"

SysIdCommand::SysIdCommand() {
    DebugOutF("inside sysidcommand constructor");
    ConfigureBindings();
}

void SysIdCommand::ConfigureBindings() {
    DebugOutF("inside sysidcommand configure bindings");

    (frc2::Trigger(BUTTON_L(8)) && Robot::GetRobot()->m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdQuasistatic(frc2::sysid::Direction::kForward)
    );

    (frc2::Trigger(BUTTON_L(6)) && Robot::GetRobot()->m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdQuasistatic(frc2::sysid::Direction::kReverse)
    );

    (frc2::Trigger(BUTTON_L(7)) && Robot::GetRobot()->m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdDynamic(frc2::sysid::Direction::kForward)
    );

    (frc2::Trigger(BUTTON_L(5)) && Robot::GetRobot()->m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdDynamic(frc2::sysid::Direction::kReverse)
    );
}

frc2::CommandPtr SysIdCommand::GetAutonomousCommand() {
    return Robot::GetRobot()->GetDriveTrain().Run([] {});
}