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
	m_Climb(CLIMB_MOTOR),
	m_ShooterMotor1(SHOOTER1_MOTOR),
	m_ShooterMotor2(SHOOTER2_MOTOR),
	m_Feeder(FEEDER_MOTOR),

	//BUTTONBOARD
	// m_TestJoystickButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(1);}),
	m_ArmOverride([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(ARM_OVERRIDE);}),
	m_ShooterUp([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(SHOOTER_UP);}),
    m_ShooterDown([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(SHOOTER_DOWN);}),
	m_RunFlywheel([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(FLYWHEEL_SWITCH);}),

    // m_ShooterDown = frc2::Trigger(BUTTON_L(SHOOTER_DOWN));
    // m_RunFlywheel = frc2::Trigger(BUTTON_L(FLYWHEEL_SWITCH));
    // m_FlywheelPowerLock = frc2::Trigger(BUTTON_L(SHOOTER_LOCK_POWER));
    // m_DustpanUp = frc2::Trigger(BUTTON_L(DUSTPAN_UP));
    // m_DustpanDown = frc2::Trigger(BUTTON_L(DUSTPAN_DOWN));
    // m_ClimbUp = frc2::Trigger(BUTTON_L(CLIMB_UP));
    // m_ClimbDown = frc2::Trigger(BUTTON_L(CLIMB_DOWN));
    // m_IntakeSwitch = frc2::Trigger(BUTTON_L(INTAKE_SWITCH));
	//m_PowerLock(BUTTON_L(SHOOTER_LOCK_POWER)),
	m_Timer()
{}

void Arm::Init() {
	DebugOutF("inside arm init");
	SetButtons();
	m_Pivot.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
	m_Feeder.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	m_Climb.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

	DebugOutF("arm override button: " + std::to_string(m_ArmOverride.Get()));
}

void Arm::SetButtons() {
	m_FlywheelPowerLock.OnTrue(new frc2::InstantCommand([&] {
		m_FlywheelPower = Robot::GetRobot()->GetButtonBoard().GetRawAxis(6);
	}));
	m_ArmOverride.OnTrue(ManualControls());
	m_RunFlywheel.OnTrue(new Flywheel());
}

// while override is active, gives manual joysticks control over the two arm motors
frc2::FunctionalCommand* Arm::ManualControls()
{
	
	return new frc2::FunctionalCommand([&] { // onInit
	}, [&] { // onExecute

		DebugOutF("inside of manual controls");
		
		// DebugOutF(std::to_string(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_UP))); //+ std::to_string(m_StringPot.GetValue() > STRINGPOT_ZERO));
		// if(m_StringPot.GetValue() > STRINGPOT_ZERO && m_StringPot.GetValue() < STRINGPOT_TOP) {
		// 	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_UP)) {
		// 		DebugOutF("inside of if for shooter up");
		// 		m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.25));
		// 	} else if(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_DOWN)) {
		// 		DebugOutF("inside of if for shooter down");
		// 		m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.25));
		// 	}
		// }
	},[&](bool e) { // onEnd
		m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
		m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
	},
	[&] { // isFinished
		return !Robot::GetRobot()->m_ArmOverride.Get();
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