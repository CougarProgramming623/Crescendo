#include "Robot.h"
#include "commands/Flywheel.h"

#define r Robot::GetRobot()

Flywheel::Flywheel() {}

void Flywheel::Initialize() {}

void Flywheel::Execute() {
    // if (r->GetArm().m_FlywheelPower != 0) {
        //r->GetArm().GetShooterMotor1().SetControl(r->m_DutyCycleOutRequest.WithOutput(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_LOCK_POWER)));
        //r->GetArm().GetShooterMotor2().SetControl(r->m_DutyCycleOutRequest.WithOutput(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_LOCK_POWER) - 0.05));
        Vision Flyvision = Robot::GetRobot()->GetVision();
        if(Flyvision.GetLimeLight()->GetNumber("tv", 0.0) == 1) {
            id = Flyvision.GetLimeLight()->GetNumber("tid", 0.0);
            dis = Flyvision.DistanceFromAprilTag(id);
            r->GetArm().m_FlywheelPower = r->GetArm().StringPotUnitsToPower(dis);
        }
        else{
            r->GetArm().GetShooterMotor1().SetControl(r->m_DutyCycleOutRequest.WithOutput(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_LOCK_POWER)));
            r->GetArm().GetShooterMotor2().SetControl(r->m_DutyCycleOutRequest.WithOutput(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_LOCK_POWER) - r->GetArm().m_Differential));
        }

        // r->GetArm().GetShooterMotor1().SetControl(r->m_DutyCycleOutRequest.WithOutput(0.2));
        // r->GetArm().GetShooterMotor2().SetControl(r->m_DutyCycleOutRequest.WithOutput(0.2 - 0.05));
    // } else {
        // r->GetArm().GetShooterMotor1().SetControl(r->m_DutyCycleOutRequest.WithOutput(0));
        // r->GetArm().GetShooterMotor2().SetControl(r->m_DutyCycleOutRequest.WithOutput(0));
    //}
}

void Flywheel::End(bool interrupted) {
    Robot::GetRobot()->GetArm().GetShooterMotor1().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    Robot::GetRobot()->GetArm().GetShooterMotor2().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
}