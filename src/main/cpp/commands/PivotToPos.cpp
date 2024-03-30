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
	//ARM.GetPivotMotor().SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
	// DebugOutF("starting at: " + std::to_string((ARM.GetPivotCANCoder().GetAbsolutePosition() - CANCODER_ZERO)) + " degrees");
	// DebugOutF("Going to: " + std::to_string(ARM.PivotTicksToDegrees(ARM.PivotDegreesToTicks(targetDegrees))) + " degrees");
	
}

void PivotToPos::Execute() {
	DebugOutF("Going into execute");
	stringpot = ARM.GetStringPot().GetAverageValue();
	int difference = stringpot - targetValue;
	double kp = 0.15;
	double targetSpeed = kp * difference;
	ARM.GetPivotMotor().Set(targetSpeed);
	// if(targetValue != stringpot) {
	// 	// if(abs(targetValue - stringpot) < 20 && abs(targetValue - stringpot) > 0 ){
	// 	// 	ARM.GetPivotMotor().Set(0.3);
	// 	// }
	// 	DebugOutF("target: " + std::to_string(targetValue));
	// 	DebugOutF("current: " + std::to_string(stringpot));
	// 	if((targetValue > (stringpot - 20)) || (targetValue > (stringpot + 20))){
	// 		ARM.GetPedivotMotor().Set(-0.7);
	// 	}
	// 	else if((targetValue < (stringpot - 20)) || (targetValue < (stringpot + 20))){
	// 		ARM.GetPivotMotor().Set(0.7);
	// 	}
	// 	else{
	// 		if (targetValue > (stringpot - 10) || targetValue > (stringpot + 10)) {
	// 			ARM.GetPivotMotor().Set(-0.1);
	// 		} else if (stringpot < (targetValue - 10) || targetValue < (stringpot + 10)) {
	// 			ARM.GetPivotMotor().Set(0.1);
	// 		} else {
	// 			ARM.GetPivotMotor().Set(0);
	// 		}
	// 	}

	// }
	// StringPotValue = ARM.StringPotLengthToStringPotUnits(ARM.PivotDegreesToStringPotLength(targetDegrees));
	// DebugOutF("Value being fed " + std::to_string(targetValue));
	// targetRotations = ARM.StringPotUnitsToRotations(targetValue);
	// ARM.GetPivotMotor().SetControl(Robot::GetRobot()->m_PositionDutyCycle.WithPosition(units::angle::turn_t(targetValue)));
	// DebugOutF(std::to_string(targetRotations));
	
	// ARM.GetPivotMotor().SetControl(Robot::GetRobot()->m_MotionMagicRequest.WithPosition(units::angle::turn_t(ARM.PivotDegreesToTicks(targetDegrees))));
	//ARM.GetPivotMotor().Set(ControlMode::MotionMagic, ARM.PivotDegreesToTicks(targetDegrees));
	//DebugOutF(std::to_string(abs(ARM.PivotDegreesToTicks(targetDegrees) - ARM.GetPivotMotor().GetSelectedSensorPosition())));
}

void PivotToPos::End(bool interrupted){
	DebugOutF("Pivot finished");
	ARM.GetPivotMotor().Set(0);
}

bool PivotToPos::IsFinished() {
	return targetValue == ARM.GetStringPot().GetAverageValue() || ARM.GetShooterUpButton().Get() || ARM.GetShooterDownButton().Get();
}