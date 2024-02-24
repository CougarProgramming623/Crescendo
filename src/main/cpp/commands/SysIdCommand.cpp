#include "commands/SysIdCommand.h"

#include "Robot.h"
#include "frc2/command/Command.h"
#include "Constants.h"

SysIdCommand::SysIdCommand() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void SysIdCommand::Initialize(){}

void SysIdCommand::Execute() {
    ConfigureBindings();
    DebugOutF("inside of sysidcommand execute");
    //both the SysId swtich and the given button must be clicked in order for the test to be run
    (frc2::Trigger(BUTTON_L(8)) && m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdQuasistatic(frc2::sysid::Direction::kForward)
    );

    (frc2::Trigger(BUTTON_L(6)) && m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdQuasistatic(frc2::sysid::Direction::kReverse)
    );

    (frc2::Trigger(BUTTON_L(7)) && m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdDynamic(frc2::sysid::Direction::kForward)
    );

    (frc2::Trigger(BUTTON_L(5)) && m_SysId).WhileTrue(
        Robot::GetRobot()->GetDriveTrain().SysIdDynamic(frc2::sysid::Direction::kReverse)
    );
}

void SysIdCommand::ConfigureBindings() {
    m_SysId = frc2::Trigger(BUTTON_L(1));
}

frc2::CommandPtr SysIdCommand::GetAutonomousCommand() {
    return Robot::GetRobot()->GetDriveTrain().Run([this] {Execute();});
}