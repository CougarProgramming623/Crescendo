#include "./commands/WristToPosAuto.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

WristToPosAuto::WristToPosAuto(double deg) {
	// targetDegrees = degPos;
	AddRequirements(&Robot::GetRobot()->GetArm());
	targetDegrees = deg;
}

void WristToPosAuto::Initialize() {
	//ARM.GetWristMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
}

void WristToPosAuto::Execute() {
	//ARM.GetWristMotor().SetPosition(units::angle::turn_t(ARM.WristStringPotUnitsToTicks(ARM.GetStringPot().GetValue())));
	//ARM.GetWristMotor().SetControl(Robot::GetRobot()->m_MotionMagicRequest.WithPosition(units::angle::turn_t(ARM.PivotDegreesToTicks(targetDegrees))));
	// DebugOutF("TargetDeg: " + std::to_string(targetDegrees));
	// DebugOutF("TargetTicks: " + std::to_string(ARM.WristDegreesToTicks(targetDegrees)));
}

void WristToPosAuto::End(bool interrupted){
	//ARM.GetWristMotor().SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(0_V));
}

bool WristToPosAuto::IsFinished() {
	//return abs(ARM.WristDegreesToTicks(targetDegrees) - ARM.GetWristMotor().GetSelectedSensorPosition()) < 20000;
	return Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
}