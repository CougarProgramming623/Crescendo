#include "Robot.h"
#include "commands/Shoot.h"

#define r Robot::GetRobot()

Shoot::Shoot() {}

void Shoot::Initialize() {}

void Shoot::Execute() {
    r->GetArm().GetDustpanLaunchServo().Set(0.75);
}

void Shoot::End(bool interrupted) {
    
}