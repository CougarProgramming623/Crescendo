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
#include "commands/ConstantPivot.h"
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
	m_ShooterUp([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_UP);}),
    m_ShooterDown([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_DOWN);}),
	m_RunFlywheel([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(FLYWHEEL_SWITCH);}),
	m_FlywheelPowerLock([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_LOCK_POWER);}),
	m_DustpanUp([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(DUSTPAN_UP);}),
	m_DustpanDown([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(DUSTPAN_DOWN);}),
	m_IntakeSwitch([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_SWITCH);}),
	m_ServoShoot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(SERVO_SHOOT);}),
	m_Aim([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(NUKE_SWITCH_4);}),
	m_CloseShootPivot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(CLOSE_SHOOT_BUTTON);}),
	m_PickupPivot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(PICKUP_BUTTON);}),
	m_ProtectedBlockPivot([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(PROTECTED_BLOCK_SHOOT);}),
	m_PivotMax([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(TEST_BIG_YELLOW_BUTTON);}),
	m_Timer()
{
	// m_Pivot.SetPosition(units::angle::turn_t(0));
}


void Arm::ArmInit() {
	DebugOutF("inside arm init");
	m_StringPot.SetAverageBits(4);

	m_Pivot.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
	m_Feeder.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	m_Aim.ToggleOnTrue(ConstantPivot().ToPtr());

	m_FlywheelPowerLock.OnTrue(new frc2::InstantCommand([&] {
		// m_FlywheelVelocity = units::turns_per_second_t((((Robot::GetRobot()->GetButtonBoard().GetRawAxis(0) + 1)/2) * 27) + 45);
		m_FlywheelVelocity = units::turns_per_second_t(72 * Robot::GetRobot()->GetButtonBoard().GetRawAxis(0));
		DebugOutF("getting the dial value, power: " + std::to_string(m_FlywheelVelocity.value()));
	}));

	

	// m_ChangeDifferential.OnTrue(new frc2::InstantCommand([&] {
	// 	// m_Differential = (Robot::GetRobot()->GetButtonBoard().GetRawAxis(1) + 1)/4;
	// 	DebugOutF("getting the dial value, differential: " + std::to_string(m_Differential));
	// }));

	m_ShooterUp.WhileTrue(std::move(frc2::InstantCommand([&] {
		if(m_StringPot.GetAverageValue() < STRINGPOT_TOP) {
			m_Pivot.Set(-0.5);
		} else {
			m_Pivot.Set(0);
		}
	})).Repeatedly()).WhileFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));
	
	
	m_ShooterDown.WhileTrue(std::move(frc2::InstantCommand([&] {
		if(m_StringPot.GetAverageValue() > STRINGPOT_LOW) {
			m_Pivot.Set(0.5);
		} else {
			m_Pivot.Set(0);
		}
	})).Repeatedly()).WhileFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));

	m_DustpanUp.OnTrue(new frc2::InstantCommand([&] {
		m_DustpanPivot.Set(0);
	}));

	m_DustpanDown.OnTrue(new frc2::InstantCommand([&] {
		m_DustpanPivot.Set(1.0 - (35.0/270.0));
	}));

	m_ServoShoot.OnTrue(new frc2::InstantCommand([&] {
		m_DustpanLaunch.Set(0.75);
	})).OnFalse(new frc2::InstantCommand([&] {
		m_DustpanLaunch.Set(1);
	}));

	m_RunFlywheel.OnTrue(new frc2::InstantCommand([&] {
		// m_ShooterMotor1.Set(m_FlywheelPower);
		// m_ShooterMotor2.Set(m_FlywheelPower * m_Differential);
		m_ShooterMotor1.SetControl(m_VelocityDutyCycle.WithVelocity(-1 * m_FlywheelVelocity * 1.05));
		m_ShooterMotor2.SetControl(m_VelocityDutyCycle.WithVelocity(m_FlywheelVelocity * m_Differential * 1.05));
	})).OnFalse(new frc2::InstantCommand([&] {
		m_ShooterMotor1.Set(0);
		m_ShooterMotor2.Set(0);
	}));

	m_IntakeSwitch.OnTrue(new frc2::InstantCommand([&] {
		m_Feeder.Set(motorcontrol::ControlMode::PercentOutput, 0.35);
	})).OnFalse(new frc2::InstantCommand([&] {
		m_Feeder.Set(motorcontrol::ControlMode::PercentOutput, 0);
	}));

	m_PivotMax.OnTrue(PivotToPos(STRINGPOT_TOP).ToPtr()).OnFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));

	m_CloseShootPivot.OnTrue(PivotToPos(CLOSEUPSHOOTSTRINGPOT).ToPtr()).OnFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));

	m_PickupPivot.OnTrue(PivotToPos(PICKUPSTRINGPOT).ToPtr()).OnFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));

	m_ProtectedBlockPivot.OnTrue(PivotToPos(PROTECTEDBLOCKSHOOT).ToPtr()).OnFalse(new frc2::InstantCommand([&] {
		m_Pivot.Set(0);
	}));
}

void Arm::Periodic() {
	Vision vision = Robot::GetRobot()->GetVision();
	if(vision.GetLimeLight()->GetNumber("tv", 0.0) == 1) {
		int id = vision.GetLimeLight()->GetNumber("tid", 0.0);
		double dis = vision.DistanceFromAprilTag(id);
		m_StringPotValue = (Robot::GetRobot()->GetArm().DistanceToStringPotUnits(dis));
		// DebugOutF("target stringpot value: " + std::to_string(m_StringPotValue));
	}
}

// while override is active, gives manual joysticks control over the two arm motors
// frc2::FunctionalCommand* Arm::ManualControls() {
// 	return new frc2::FunctionalCommand([&] { // onInit
// 	}, [&] { // onExecute
// 		DebugOutF("inside of manual controls");
// 		if(m_ShooterUp.Get()) {

// 		} else {
// 			m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
// 		}

// 		if(m_ShooterDown.Get()) {
// 			m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
// 		} else {
// 			m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
// 		}
// 	},[&](bool e) { // onEnd
// 		m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
// 	},
// 	[&] { // isFinished
// 		return m_ArmOverride.Get();
// 	});
// }

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