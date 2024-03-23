#include "subsystems/Arm.h"
#include "Robot.h"
#include "frc2/command/PrintCommand.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "commands/Flywheel.h"
#include "commands/Intake.h"
#include "Constants.h"
#include "commands/AutoTest.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix6;


Arm::Arm(): 
	m_Pivot(PIVOT_MOTOR),
	m_ShooterMotor1(SHOOTER1_MOTOR),
	m_ShooterMotor2(SHOOTER2_MOTOR),
	m_Feeder(FEEDER_MOTOR),

	//BUTTONBOARD
	// m_TestJoystickButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(1);}),
	m_ArmOverride([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);}),
	m_ShooterUp([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_UP);}),
    m_ShooterDown([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_DOWN);}),
	m_RunFlywheel([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(FLYWHEEL_SWITCH);}),
	m_FlywheelPowerLock([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_LOCK_POWER);}),
	m_DustpanUp([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(DUSTPAN_UP);}),
	m_DustpanDown([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(DUSTPAN_DOWN);}),
	m_IntakeSwitch([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_SWITCH);}),
	m_ServoShoot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SERVO_SHOOT);}),
	m_Aim([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(AIM_BUTTON);}),
	m_CloseShootPivot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(CLOSE_SHOOT_BUTTON);}),
	m_PickupPivot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(PICKUP_BUTTON);}),
	m_ProtectedBlockPivot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(PROTECTED_BLOCK_SHOOT);}),
	m_Timer()
{
	// m_Pivot.SetPosition(units::angle::turn_t(0));
}

void Arm::ArmInit() {
	DebugOutF("inside arm init");
	m_StringPot.SetAverageBits(3);

	m_Pivot.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
	m_Feeder.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	m_FlywheelPowerLock.OnTrue(new frc2::InstantCommand([&] {
		DebugOutF("getting the dial value");
		m_FlywheelPower = Robot::GetRobot()->GetButtonBoard().GetRawAxis(6);
	}));

	m_ShooterUp.WhileTrue(new frc2::InstantCommand([&] {
		if(m_StringPot.GetValue() < STRINGPOT_TOP) {
			m_Pivot.Set(-0.5);
		} else {
			m_Pivot.Set(0);
		}
	})).WhileFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));
	
	
	m_ShooterDown.WhileTrue(new frc2::InstantCommand([&] {
		if(m_StringPot.GetValue() > STRINGPOT_LOW) {
			m_Pivot.Set(0.5);
		} else {
			m_Pivot.Set(0);
		}
	})).WhileFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));

	m_DustpanUp.OnTrue(new frc2::InstantCommand([&] {
		m_DustpanPivot.Set(0);
	}));

	m_DustpanDown.OnTrue(new frc2::InstantCommand([&] {
		m_DustpanPivot.Set(1);
	}));

	m_ServoShoot.OnTrue(new frc2::InstantCommand([&] {
		m_DustpanLaunch.Set(0.75);
	})).OnFalse(new frc2::InstantCommand([&] {
		m_DustpanLaunch.Set(1);
	}));

	// m_ArmOverride.OnTrue(ManualControls());

	m_RunFlywheel.OnTrue(new frc2::InstantCommand([&] {
		m_ShooterMotor1.Set(m_FlywheelPower);
		m_ShooterMotor2.Set(m_FlywheelPower - 0.1);
	})).OnFalse(new frc2::InstantCommand([&] {
		m_ShooterMotor1.Set(0);
		m_ShooterMotor2.Set(0);
	}));

	m_IntakeSwitch.OnTrue(new frc2::InstantCommand([&] {
		m_Feeder.Set(motorcontrol::ControlMode::PercentOutput, 0.35);
	})).OnFalse(new frc2::InstantCommand([&] {
		m_Feeder.Set(motorcontrol::ControlMode::PercentOutput, 0);
	}));
	
	// m_Aim.OnTrue(new frc2::InstantCommand([&] {
	// 	DebugOutF("current value: " + std::to_string(GetStringPot().GetValue()));
	// 	double 
	// 	MoveToStringPotValue(450);
	// })).OnFalse(new frc2::InstantCommand([&] {
	// 	m_Pivot.Set(0);
	// }));

	// m_CloseShootPivot.OnTrue(new frc2::InstantCommand([&] {
	// 	DebugOutF("current value: " + std::to_string(GetStringPot().GetValue()));
	// 	MoveToStringPotValue(555);
	// })).OnFalse(new frc2::InstantCommand([&] {
	// 	m_Pivot.Set(0);
	// }));

	// m_PickupPivot.OnTrue(new frc2::InstantCommand([&] {
	// 	DebugOutF("current value: " + std::to_string(GetStringPot().GetValue()));
	// 	MoveToStringPotValue(420);
	// })).OnFalse(new frc2::InstantCommand([&] {
	// 	m_Pivot.Set(0);
	// }));

	m_CloseShootPivot.OnTrue(PivotToPos(555).ToPtr()).OnFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));

	m_ProtectedBlockPivot.OnTrue(PivotToPos(420).ToPtr()).OnFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));

	// m_ProtectedBlockPivot.OnTrue(new frc2::InstantCommand([&] {
	// 	DebugOutF("current value: " + std::to_string(GetStringPot().GetValue()));
	// 	MoveToStringPotValue(idk); //NEEDS TO BE LOOKED AT
	// })).OnFalse(new frc2::InstantCommand([&] {
	// 	m_Pivot.Set(0);
	// }));

}
	int Arm::ConvertDistanceToValue() {
		Vision vision = Robot::GetRobot()->GetVision();
		if(vision.GetLimeLight()->GetNumber("tv", 0.0) == 1) {
			int d = vision.DistanceFromAprilTag(Robot::GetRobot()->GetVision().GetLimeLight()->GetNumber("tid", 0.0));
			int val = int(1 * pow(d,4) + 1 * pow(d,3) + 1 * pow(d,2) + 1 * d + 3);
			return val;
		}
	}
	void Arm::MoveToStringPotValue(int target){
		while(abs(target - GetStringPot().GetValue()) > 5) {
			// DebugOutF(std::to_string(StringPotUnitsToRotations(GetStringPot().GetValue())));
			// DebugOutF("stringpot value: " + std::to_string(GetStringPot().GetValue()));
			if((target > GetStringPot().GetValue() - 5) || (target > GetStringPot().GetValue() + 5)){
				m_Pivot.Set(-1);
			}
			else if((target < GetStringPot().GetValue() - 5) || (target > GetStringPot().GetValue() + 5)){
				m_Pivot.Set(1);
			}
			if(abs(target - GetStringPot().GetValue()) < 5){
				break;
			}
			if(abs(target - GetStringPot().GetValue()) < 20 && abs(target - GetStringPot().GetValue()) > 0 ){
				m_Pivot.Set(0.3);
			}
		}
	}

// while override is active, gives manual joysticks control over the two arm motors
frc2::FunctionalCommand* Arm::ManualControls()
{
	
	return new frc2::FunctionalCommand([&] { // onInit
	}, [&] { // onExecute
		DebugOutF("inside of manual controls");
		if(m_ShooterUp.Get()) {
			
		} else {
			m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
		}

		if(m_ShooterDown.Get()) {
			m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
		} else {
			m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
		}
	},[&](bool e) { // onEnd
		m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
	},
	[&] { // isFinished
		return m_ArmOverride.Get();
	});
}

// void Arm::SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc) {
// 	//LOOK
// 	configs::MotionMagicConfigs pivotMotionMagicConfigs;
// 	configs::MotionMagicConfigs wristMotionMagicConfigs;
// 	pivotMotionMagicConfigs.WithMotionMagicCruiseVelocity(pivotVel);
// 	pivotMotionMagicConfigs.WithMotionMagicAcceleration(pivotAcc);
// 	wristMotionMagicConfigs.WithMotionMagicCruiseVelocity(wristVel);
// 	wristMotionMagicConfigs.WithMotionMagicAcceleration(wristAcc);
// 	m_Pivot.GetConfigurator().Apply(pivotMotionMagicConfigs, 0_s);
// 	m_Wrist.GetConfigurator().Apply(wristMotionMagicConfigs, 0_s);
// 	m_Pivot.GetConfigurator().Apply(pivotMotionMagicConfigs, 0_s);
// 	m_Wrist.GetConfigurator().Apply(wristMotionMagicConfigs, 0_s);
// }