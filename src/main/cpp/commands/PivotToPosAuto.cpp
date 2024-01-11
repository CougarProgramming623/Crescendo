#include "./commands/PivotToPosAuto.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

PivotToPosAuto::PivotToPosAuto(double deg) {
	// targetDegrees = degPos;
	// AddRequirements(&Robot::GetRobot()->GetArm());
	targetDegrees = deg;

}

void PivotToPosAuto::Initialize() {
	ARM.GetPivotMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
	// DebugOutF("starting at: " + std::to_string((ARM.GetPivotCANCoder().GetAbsolutePosition() - CANCODER_ZERO)) + " degrees");
	// DebugOutF("Going to: " + std::to_string(ARM.PivotTicksToDegrees(ARM.PivotDegreesToTicks(targetDegrees))) + " degrees");
}

void PivotToPosAuto::Execute() {
	ARM.GetPivotMotor().Set(ControlMode::MotionMagic, ARM.PivotDegreesToTicks(targetDegrees));
	//DebugOutF(std::to_string(abs(ARM.PivotDegreesToTicks(targetDegrees) - ARM.GetPivotMotor().GetSelectedSensorPosition())));
}

void PivotToPosAuto::End(bool interrupted){
	DebugOutF("Pivot finished");
	ARM.GetPivotMotor().Set(ControlMode::PercentOutput, 0);
}

bool PivotToPosAuto::IsFinished() {
	return Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE) ;
	//return abs(ARM.PivotDegreesToTicks(targetDegrees) - ARM.GetPivotMotor().GetSelectedSensorPosition()) < 4000;
}