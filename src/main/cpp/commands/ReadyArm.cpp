#include "Robot.h"
#include "commands/ReadyArm.h"

#define r Robot::GetRobot()

ReadyArm::ReadyArm() {}

void ReadyArm::Initialize() {}

void ReadyArm::Execute() {
  PivotToPos(PICKUPSTRINGPOT).ToPtr();
}

void ReadyArm::End(bool interrupted) {}




