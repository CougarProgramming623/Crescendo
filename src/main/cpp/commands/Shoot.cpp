#include "Robot.h"
#include "commands/Shoot.h"

#define r Robot::GetRobot()

Shoot::Shoot() {}

void Shoot::Initialize() {}

void Shoot::Execute() {
    // if (r->GetArm().GetDustpanLaunchServo().Get() == 0.75) {
    //     r->GetArm().GetDustpanLaunchServo().Set(1);
    // } else if (r->GetArm().GetDustpanLaunchServo().Get() == 1) {
    //     r->GetArm().GetDustpanLaunchServo().Set(0.75);
    // }

    frc2::WaitCommand(2.0_s);
    r->GetArm().GetDustpanLaunchServo().Set(0.75);
}

void Shoot::End(bool interrupted) {
    
}