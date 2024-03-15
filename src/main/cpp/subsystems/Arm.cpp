#include "subsystems/Arm.h"
#include "Robot.h"
#include "frc2/command/PrintCommand.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "commands/Flywheel.h"
#include "Constants.h"
//#include <ctre/phoenix6/configs/Configs.hpp>

using ctre::phoenix::motorcontrol::NeutralMode;
using namespace ctre::phoenix;
using ctre::phoenix::motorcontrol::can::TalonSRX;
using namespace ctre::phoenix6;
//using ctre::phoenix6::configs::MagnetSensorConfigs;
// using ctre::phoenix6::signals::AbsoluteSensorRangeValue;



Arm::Arm(): 
	m_Pivot(PIVOT_MOTOR),
	m_Climb(CLIMB_MOTOR),
	m_ShooterMotor1(SHOOTER1_MOTOR),
	m_ShooterMotor2(SHOOTER2_MOTOR),
	m_Feeder(FEEDER_MOTOR),

	//BUTTONBOARD
	m_ArmOverride(BUTTON_L(ARM_OVERRIDE)),
	m_ShooterUp(BUTTON_L(SHOOTER_UP)),
	m_ShooterDown(BUTTON_L(SHOOTER_DOWN)),
	m_RunFlywheel(BUTTON_L(FLYWHEEL_SWITCH)),
	m_FlywheelPowerLock(BUTTON_L(SHOOTER_LOCK_POWER)),
	m_DustpanUp(BUTTON_L(DUSTPAN_UP)),
	m_DustpanDown(BUTTON_L(DUSTPAN_DOWN)),
	m_ClimbUp(BUTTON_L(CLIMB_UP)),
	m_ClimbDown(BUTTON_L(CLIMB_DOWN)),
	
	m_Timer()
{}

void Arm::Init() {
	SetButtons();
	DebugOutF("inside arm init");
	// m_Pivot.SetNeutralMode(NeutralMode::Brake);
	// m_Feeder.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	// m_Climb.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);

	DebugOutF("arm override button: " + std::to_string(Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE)));

	// if (Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE)) {
	// 	DebugOutF("testest");
	// }
}

void Arm::SetButtons() {
	m_FlywheelPowerLock.OnTrue(new frc2::InstantCommand([&] {
		m_FlywheelPower = Robot::GetRobot()->GetButtonBoard().GetRawAxis(6);
	}));
	m_ArmOverride.OnTrue(ManualControls());
	m_RunFlywheel.OnTrue(new Flywheel());

	// m_GroundPickupMode.OnTrue(new frc2::InstantCommand([&]{
	// 	//Robot::GetRobot()->GetArm().m_PivotPos = 98.0;
    //   	//Robot::GetRobot()->GetArm().m_WristPos = 3.0;
	// 	//SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
	// 	/*new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand("Ground Pickup"),
	// 		//PivotToPos(), 
    //   		//WristToPos()
	//   	);*/
	// }));

	// m_TransitMode.OnTrue(new frc2::InstantCommand([&]{
	// 	//Robot::GetRobot()->GetArm().m_PivotPos = 66.6;
    //   	//Robot::GetRobot()->GetArm().m_WristPos = 132.0;
	// 	SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
	// 	/*new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand(""),
	// 		//PivotToPos(), 
    //   		//WristToPos()
	//   	);*/
	// }));

	// m_GroundPickupMode.WhenPressed(
	// 	new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand("-45"),
	// 		PivotToPos(98.0)
	// 	)
	// 	// new WristToPos(WRIST_GROUND_ANGLE)
	// );

	// m_TransitMode.WhenPressed(
	// 	new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand("0"),
	// 		PivotToPos(0)
	// 	)		// new WristToPos(WRIST_TRANSIT_ANGLE)
	// );

	// m_PlacingMode.WhenPressed(
	// 	new frc2::ParallelCommandGroup(
	// 		frc2::PrintCommand("45"),
	// 		PivotToPos(45)
	// 	)		// new WristToPos(WRIST_PLACING_MID_CUBE_ANGLE)
	// );	
}

// while override is active, gives manual joysticks control over the two arm motors
frc2::FunctionalCommand* Arm::ManualControls()
{
	
	return new frc2::FunctionalCommand([&] { // onInit
	}, [&] { // onExecute

		DebugOutF("inside of manual controls");
		
		DebugOutF(std::to_string(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_UP))); //+ std::to_string(m_StringPot.GetValue() > STRINGPOT_ZERO));
		// if(m_StringPot.GetValue() > STRINGPOT_ZERO && m_StringPot.GetValue() < STRINGPOT_TOP) {
		// 	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_UP)) {
		// 		DebugOutF("inside of if for shooter up");
		// 		m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.25));
		// 	} else if(Robot::GetRobot()->GetButtonBoard().GetRawButton(SHOOTER_DOWN)) {
		// 		DebugOutF("inside of if for shooter down");
		// 		m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.25));
		// 	}
		// }
		// m_ShooterMotor1.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.6));
		// m_ShooterMotor2.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.55));

	},[&](bool e) { // onEnd
		// m_Pivot.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
		// m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
	},
	[&] { // isFinished
		return !Robot::GetRobot()->m_ArmOverride.Get();
		return false;
	});
}

void Arm::SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc) {
	//LOOK
	configs::MotionMagicConfigs pivotMotionMagicConfigs;
	configs::MotionMagicConfigs wristMotionMagicConfigs;
	pivotMotionMagicConfigs.WithMotionMagicCruiseVelocity(pivotVel);
	pivotMotionMagicConfigs.WithMotionMagicAcceleration(pivotAcc);
	wristMotionMagicConfigs.WithMotionMagicCruiseVelocity(wristVel);
	wristMotionMagicConfigs.WithMotionMagicAcceleration(wristAcc);
	// m_Pivot.GetConfigurator().Apply(pivotMotionMagicConfigs, 0_s);
	// m_Wrist.GetConfigurator().Apply(wristMotionMagicConfigs, 0_s);
}