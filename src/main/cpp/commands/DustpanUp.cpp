#include "Robot.h"
#include "commands/DustpanUp.h"

#define r Robot::GetRobot()

DustpanUp::DustpanUp() {}

void DustpanUp::Initialize() {}

void DustpanUp::Execute() {
  r->GetArm().GetDustpanPivotServo().Set(0);
}

void DustpanUp::End(bool interrupted) {}

bool DustpanUp::IsFinished() {
  return r->GetArm().GetDustpanPivotServo().Get() == 0;
}