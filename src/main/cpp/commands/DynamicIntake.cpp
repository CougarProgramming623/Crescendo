#include "./commands/DynamicIntake.h"
#include "Robot.h"


#define ARM Robot::GetRobot()->GetArm()

DynamicIntake::DynamicIntake() {
	AddRequirements(&Robot::GetRobot()->m_Intake);
}

void DynamicIntake::Initialize() {

}

void DynamicIntake::Execute() {
	// double power = .55;

	// if(Robot::GetRobot()->GetButtonBoard().GetRawButton(CUBE_MODE)) {
	// 	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) {
	// 		ARM.GetTopIntakeMotor().Set(ControlMode::PercentOutput, power);
	// 		ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, power);
	// 	} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) {
	// 		ARM.GetTopIntakeMotor().Set(ControlMode::PercentOutput, -power);
	// 		ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -power);
	// 	}
	// } else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(CONE_MODE)) {
	// 	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) {
	// 		ARM.GetTopIntakeMotor().Set(ControlMode::PercentOutput, power);
	// 		ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -power);
	// 	} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) {
	// 		ARM.GetTopIntakeMotor().Set(ControlMode::PercentOutput, -power);
	// 		ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, power);
	// 	}
	// }

	// ---------------------------------------------------------------------

	double power = -.7; //default power for cone
	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)){
		ARM.GetBottomIntakeMotor().EnableCurrentLimit(true);
		ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, power);
	} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)){
		ARM.GetBottomIntakeMotor().EnableCurrentLimit(false);
		ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -power);
	} else 
		ARM.GetBottomIntakeMotor().EnableCurrentLimit(true);

}

void DynamicIntake::End(bool interrupted){
	ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
	// ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
}

bool DynamicIntake::IsFinished() {
	return !Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON) &&
		   !Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON) &&
		   !Robot::GetRobot()->GetJoyStick().GetRawButton(4);
}