#include "Robot.h"
#include "commands/Flywheel.h"

Flywheel::Flywheel() {}

void Flywheel::Initialize() {}

void Flywheel::Execute() {
    Robot* r = Robot::GetRobot();
    if (r->GetArm().m_FlywheelPower > 0) {
        // r->GetArm().GetShooterMotor1().SetControl(r->m_DutyCycleOutRequest.WithOutput(r->GetArm().m_FlywheelPower));
        // r->GetArm().GetShooterMotor2().SetControl(r->m_DutyCycleOutRequest.WithOutput(r->GetArm().m_FlywheelPower * r->GetArm().m_Differential));
        r->GetArm().GetShooterMotor1().SetControl(r->GetArm().m_VelocityDutyCycle.WithVelocity(units::turns_per_second_t(r->GetArm().m_FlywheelPower * 75)));
        r->GetArm().GetShooterMotor2().SetControl(r->GetArm().m_VelocityDutyCycle.WithVelocity(units::turns_per_second_t(r->GetArm().m_FlywheelPower * 75 * r->GetArm().m_Differential)));
    } else {
        r->GetArm().GetShooterMotor1().Set(0);
        r->GetArm().GetShooterMotor2().Set(0);
    }
}

void Flywheel::End(bool interrupted) {
    Robot::GetRobot()->GetArm().GetShooterMotor1().Set(0);
    Robot::GetRobot()->GetArm().GetShooterMotor2().Set(0);
}