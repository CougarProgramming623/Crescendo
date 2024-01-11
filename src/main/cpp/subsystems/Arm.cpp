#include "subsystems/Arm.h"
#include "Robot.h"
#include "frc2/command/PrintCommand.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "./commands/DriveToPosCommand.h"
#include "Constants.h"

using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::can::TalonSRX;
using ctre::phoenix::sensors::AbsoluteSensorRange;



Arm::Arm() : m_Pivot(PIVOT_MOTOR),
			 m_Wrist(WRIST_MOTOR),
			//  m_TopIntake(TOP_INTAKE_MOTOR),
			 m_BottomIntake(BOTTOM_INTAKE_MOTOR/*, rev::CANSparkMaxLowLevel::MotorType::kBrushless*/),

			 //BUTTONBOARD 1
			 m_Override(BUTTON_L(ARM_OVERRIDE)),
			 m_Override2(BUTTON_L(ARM_OVERRIDE_2)),

			 m_ConeMode(BUTTON_L(CONE_MODE)),  
			 m_CubeMode(BUTTON_L(CUBE_MODE)),

			 m_IntakeButton(BUTTON_L(INTAKE_BUTTON)),
			 m_OuttakeButton(BUTTON_L(OUTTAKE_BUTTON)),

			 

			m_Timer()


			// m_Top(PlaceElement(0, 2)),

			// m_Mid(PlaceElement(1, 2)),

			// m_Bot(PlaceElement(2, 2))
			{}

void Arm::Init()
{
	SetButtons();

	m_Pivot.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	m_Wrist.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	// m_TopIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	// m_BottomIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	m_Pivot.ConfigAllowableClosedloopError(0, 10);
	m_Pivot.Config_kP(0, 0.01);
	m_Pivot.Config_kI(0, 0.00002); //0.000005 || 0.00001 original on 4/2/23
	m_Pivot.Config_kD(0, 0.4); //0.3 original on 4/2/23
	m_Pivot.Config_kF(0, 0.0639375, 0);

	m_Wrist.ConfigAllowableClosedloopError(0, 0);
	m_Wrist.Config_kP(0, 0.0175); //0.009
	m_Wrist.Config_kI(0, 0.000002);
	m_Wrist.Config_kD(0, 0.6);
	m_Wrist.Config_kF(0, 0.02, 0); //0.06089285714
	
	SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);

	

	// m_TopIntake.ConfigPeakCurrentDuration(1750);
	// m_TopIntake.ConfigPeakCurrentLimit(6);
	// m_TopIntake.ConfigContinuousCurrentLimit(2);
	// m_TopIntake.EnableCurrentLimit(true);
	// m_BottomIntake.ConfigPeakCurrentDuration(1750);
	// m_BottomIntake.ConfigPeakCurrentLimit(7);
	// m_BottomIntake.ConfigContinuousCurrentLimit(3.5);
	// m_BottomIntake.EnableCurrentLimit(true);
 

	m_PivotCANCoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
	m_Pivot.SetSelectedSensorPosition((CANCODER_ZERO - m_PivotCANCoder.GetAbsolutePosition()) * PIVOT_TICKS_PER_DEGREE);
	m_Wrist.SetSelectedSensorPosition((WristStringPotUnitsToTicks(m_StringPot.GetValue())));
}

void Arm::SetButtons()
{
	m_Override.WhenPressed(ManualControls());

	m_IntakeButton.WhenPressed(DynamicIntake());
	m_OuttakeButton.WhenPressed(DynamicIntake());

	m_GroundPickupMode.WhenPressed(new frc2::InstantCommand([&]{
		Robot::GetRobot()->GetArm().m_PivotPos = 98.0;
      	Robot::GetRobot()->GetArm().m_WristPos = 3.0;
		SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
		new frc2::ParallelCommandGroup(
			frc2::PrintCommand("Ground Pickup"),
			PivotToPos(), 
      		WristToPos()
	  	);
	}));

	m_TransitMode.WhenPressed(new frc2::InstantCommand([&]{
		Robot::GetRobot()->GetArm().m_PivotPos = 66.6;
      	Robot::GetRobot()->GetArm().m_WristPos = 132.0;
		SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
		new frc2::ParallelCommandGroup(
			frc2::PrintCommand(""),
			PivotToPos(), 
      		WristToPos()
	  	);
	}));

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
		SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
	}, [&] { // onExecute
		m_Pivot.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetButtonBoard().GetRawAxis(PIVOT_CONTROL) / 2);
		m_Wrist.Set(ControlMode::PercentOutput, Robot::GetRobot()->GetButtonBoard().GetRawAxis(WRIST_CONTROL) / 2);

	// ---------------------------------------------------------------------------------------

	//  double power = .55;
	// 	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(CUBE_MODE)) {
	// 		if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) {
	// 			m_TopIntake.Set(ControlMode::PercentOutput, power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, power);
	// 		} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) {
	// 			m_TopIntake.Set(ControlMode::PercentOutput, -power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, -power);
	// 		}
	// 	} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(CONE_MODE)) {
	// 		if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) {f
	// 			m_TopIntake.Set(ControlMode::PercentOutput, power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, -power);
	// 		} else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) {
	// 			m_TopIntake.Set(ControlMode::PercentOutput, -power);
	// 			m_BottomIntake.Set(ControlMode::PercentOutput, power);
	// 		}
	// }

	//---------------------------------------------------------------------------------------------
	
	double power = -.7; 
	
	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) m_BottomIntake.Set(ControlMode::PercentOutput, power);
	else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) m_BottomIntake.Set(ControlMode::PercentOutput, 1);
	else m_BottomIntake.Set(ControlMode::PercentOutput, 0);

	},[&](bool e) { // onEnd
		m_Pivot.Set(ControlMode::PercentOutput, 0);
		m_Wrist.Set(ControlMode::PercentOutput, 0);
		m_BottomIntake.Set(ControlMode::PercentOutput, 0);
	},
	[&] { // isFinished
		return !Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
	});
}

void Arm::SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc) {
	m_Pivot.ConfigMotionCruiseVelocity(pivotVel, 0); 
	m_Pivot.ConfigMotionAcceleration(pivotAcc, 0); 
	m_Wrist.ConfigMotionCruiseVelocity(wristVel, 0); 
	m_Wrist.ConfigMotionAcceleration(wristAcc, 0);
}