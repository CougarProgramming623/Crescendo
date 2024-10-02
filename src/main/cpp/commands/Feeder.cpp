#include "Robot.h"
#include "commands/Feeder.h"

#define r Robot::GetRobot()

Feeder::Feeder() {}

void Feeder::Initialize() {}

void Feeder::Execute() {
  r->GetArm().GetFeeder().Set(motorcontrol::ControlMode::PercentOutput, 0.35);
}

void Feeder::End(bool interrupted) {}




