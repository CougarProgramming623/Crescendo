#include "./commands/WristToPos.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

WristToPos::WristToPos() {
	// targetDegrees = degPos;
	AddRequirements(&Robot::GetRobot()->GetArm());
}

void WristToPos::Initialize() {
	ARM.GetWristMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
	targetDegrees = Robot::GetRobot()->GetArm().m_WristPos;
}

void WristToPos::Execute() {
	ARM.GetWristMotor().SetSelectedSensorPosition((ARM.WristStringPotUnitsToTicks(ARM.GetStringPot().GetValue())));
	ARM.GetWristMotor().Set(ControlMode::MotionMagic, ARM.WristDegreesToTicks(targetDegrees));
	// DebugOutF("TargetDeg: " + std::to_string(targetDegrees));
	// DebugOutF("TargetTicks: " + std::to_string(ARM.WristDegreesToTicks(targetDegrees)));
	


}

void WristToPos::End(bool interrupted){
	ARM.GetWristMotor().Set(ControlMode::PercentOutput, 0);
}

bool WristToPos::IsFinished() {
	//return abs(ARM.WristDegreesToTicks(targetDegrees) - ARM.GetWristMotor().GetSelectedSensorPosition()) < 20000;
	return Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
}