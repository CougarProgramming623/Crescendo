#include "commands/PivotToPos.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

PivotToPos::PivotToPos(int target) {
	DebugOutF("Going into constructor");
	targetValue = target;
	AddRequirements(&Robot::GetRobot()->GetArm());
}

void PivotToPos::Initialize() {
	DebugOutF("Going into init");
}

void PivotToPos::Execute() {
	DebugOutF("Going into execute");
	stringpot = ARM.GetStringPot().GetAverageValue();
	int difference = stringpot - targetValue;
	double kp = 0.15;
	double targetSpeed = kp * difference;
	ARM.GetPivotMotor().Set(targetSpeed);
}

void PivotToPos::End(bool interrupted){
	DebugOutF("Pivot finished");
	ARM.GetPivotMotor().Set(0);
}

bool PivotToPos::IsFinished() {
	return targetValue == ARM.GetStringPot().GetAverageValue() || ARM.GetShooterUpButton().Get() || ARM.GetShooterDownButton().Get();
}