#include "Robot.h"
#include "commands/DustpanDown.h"

#define r Robot::GetRobot()

DustpanDown::DustpanDown() {}

void DustpanDown::Initialize() {}

void DustpanDown::Execute() {
  r->GetArm().GetDustpanPivotServo().Set(1.0 - (35.0/270.0));
}

void DustpanDown::End(bool interrupted) {}

bool DustpanDown::IsFinished(){
  return r->GetArm().GetDustpanPivotServo().Get() == (1.0 - (35.0/270.0));
}