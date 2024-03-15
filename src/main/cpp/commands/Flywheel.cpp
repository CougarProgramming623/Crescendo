#include "Robot.h"
#include "commands/Flywheel.h"

#define r Robot::GetRobot()

Flywheel::Flywheel() {}

void Flywheel::Initialize() {}

void Flywheel::Execute() {
    if (r->GetArm().m_FlywheelPower != 0) {
        r->GetArm().GetShooterMotor1().SetControl(r->m_DutyCycleOutRequest.WithOutput(r->GetArm().m_FlywheelPower));
        r->GetArm().GetShooterMotor2().SetControl(r->m_DutyCycleOutRequest.WithOutput(r->GetArm().m_FlywheelPower - 0.05));
    } else {
        r->GetArm().GetShooterMotor1().SetControl(r->m_DutyCycleOutRequest.WithOutput(0));
        r->GetArm().GetShooterMotor2().SetControl(r->m_DutyCycleOutRequest.WithOutput(0));
    }

}

void Flywheel::End(bool interrupted) {
    Robot::GetRobot()->GetArm().GetShooterMotor1().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    Robot::GetRobot()->GetArm().GetShooterMotor2().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
}