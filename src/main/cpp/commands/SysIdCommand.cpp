#include "commands/SysIdCommand.h"

#include "Robot.h"
#include "frc2/command/Command.h"

SysIdCommand::SysIdCommand() {
    ConfigureBindings();
}

void SysIdCommand::ConfigureBindings() {
    m_DriveTrain.SetDefaultCommand(DriveWithJoystick());
    m_SysId = frc2::Trigger(BUTTON_L(1));

    //both the SysId swtich and the given button must be clicked in order for the test to be run
    (/*FIRST OF FOUR BUTTONS AT THE TOP*/ && m_SysId).WhileTrue(
        m_DriveTrain.SysIdQuasistatic(frc2::sysid::Direction::kForward)
    );

    (/*SECOND OF FOUR BUTTONS AT THE TOP*/ && m_SysId).WhileTrue(
        m_DriveTrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse)
    );

    (/*THIRD OF FOUR BUTTONS AT THE TOP*/ && m_SysId).WhileTrue(
        m_DriveTrain.SysIdDynamic(frc2::sysid::Direction::kForward)
    );

    (/*FOURTH OF FOUR BUTTONS AT THE TOP*/ && m_SysId).WhileTrue(
        m_DriveTrain.SysIdDynamic(frc2::sysid::Direction::kReverse)
    );
}

frc2::CommandPtr SysIdCommand::GetAutonomousCommand() {
    return m_DriveTrain.Run([] {});
}