#include "./commands/PivotToPos.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

PivotToPos::PivotToPos() {
	// targetDegrees = degPos;
	// AddRequirements(&Robot::GetRobot()->GetArm());

}

void PivotToPos::Initialize() {
	ARM.GetPivotMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
	targetDegrees = Robot::GetRobot()->GetArm().m_PivotPos;
	// DebugOutF("starting at: " + std::to_string((ARM.GetPivotCANCoder().GetAbsolutePosition() - CANCODER_ZERO)) + " degrees");
	// DebugOutF("Going to: " + std::to_string(ARM.PivotTicksToDegrees(ARM.PivotDegreesToTicks(targetDegrees))) + " degrees");
}

void PivotToPos::Execute() {
	ARM.GetPivotMotor().Set(ControlMode::MotionMagic, ARM.PivotDegreesToTicks(targetDegrees));
	//DebugOutF(std::to_string(abs(ARM.PivotDegreesToTicks(targetDegrees) - ARM.GetPivotMotor().GetSelectedSensorPosition())));
}

void PivotToPos::End(bool interrupted){
	DebugOutF("Pivot finished");
	ARM.GetPivotMotor().Set(ControlMode::PercentOutput, 0);
}

bool PivotToPos::IsFinished() {
	return Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE) ;
	//return abs(ARM.PivotDegreesToTicks(targetDegrees) - ARM.GetPivotMotor().GetSelectedSensorPosition()) < 4000;
}