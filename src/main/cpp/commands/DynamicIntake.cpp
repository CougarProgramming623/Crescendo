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
	//current limiter configuration obj
	configs::CurrentLimitsConfigs bottomIntakeCurrentConfigs{};
	ARM.GetBottomIntakeMotor().GetConfigurator().Apply(bottomIntakeCurrentConfigs);
	
	//was using Robot::GetRobot()->m_VoltageOutRequest to access the voltage request, 
	//below lines were changed to work similar to arm.cpp intake buttons

	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)){
		bottomIntakeCurrentConfigs.SupplyCurrentLimitEnable = true;
		ARM.GetBottomIntakeMotor().SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(units::voltage::volt_t(power * 12)));
		//ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, power);
	} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)){
		bottomIntakeCurrentConfigs.SupplyCurrentLimitEnable = false;
		//not sure what happens when you negate the voltage, im assuming another method must be called to inverse the output of the recieved voltage
		ARM.GetBottomIntakeMotor().SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(units::voltage::volt_t(-power * 12)));
		//ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -power);
	} else 
		bottomIntakeCurrentConfigs.SupplyCurrentLimitEnable = true;

}

void DynamicIntake::End(bool interrupted){
	ARM.GetBottomIntakeMotor().SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(0_V));
	// ARM.GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
}

bool DynamicIntake::IsFinished() {
	return !Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON) &&
		   !Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON) &&
		   !Robot::GetRobot()->GetJoyStick().GetRawButton(4);
}