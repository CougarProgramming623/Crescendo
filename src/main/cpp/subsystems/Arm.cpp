#include "subsystems/Arm.h"
#include "Robot.h"
#include "frc2/command/PrintCommand.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
//#include "./commands/DriveToPosCommand.h"
#include "Constants.h"
//#include <ctre/phoenix6/configs/Configs.hpp>

// using ctre::phoenix::motorcontrol::ControlMode;
// using ctre::phoenix::motorcontrol::can::TalonFX;
// using ctre::phoenix::motorcontrol::can::TalonSRX;
using namespace ctre::phoenix6;
//using ctre::phoenix6::configs::MagnetSensorConfigs;
// using ctre::phoenix6::signals::AbsoluteSensorRangeValue;



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

	m_Pivot.SetNeutralMode(signals::NeutralModeValue::Brake);
	m_Wrist.SetNeutralMode(signals::NeutralModeValue::Brake);
	// m_TopIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
	// m_BottomIntake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

	// set slot 0 PID
	configs::Slot0Configs pivotConfigs{};
	pivotConfigs.kP = 0.01;
	pivotConfigs.kI = 0.00002; //0.000005 || 0.00001 original on 4/2/23
	pivotConfigs.kD = 0.4; //0.3 original on 4/2/23
	pivotConfigs.kV = 0.0639375;
	//apply slot 0 PID
	m_Pivot.GetConfigurator().Apply(pivotConfigs, 50_ms);

	//couldn't find equivalent method in phoenix6, not sure how important
	//m_Pivot.ConfigAllowableClosedloopError(0, 10);

	configs::Slot0Configs wristConfigs{};
	wristConfigs.kP = 0.0175; //0.009
	wristConfigs.kI = 0.000002;
	wristConfigs.kD = 0.6;
	wristConfigs.kV = 0.02; //0.06089285714

	//couldn't find equivalent method in phoenix6, not sure how important
	//m_Wrist.ConfigAllowableClosedloopError(0, 0);
	
	SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);

	

	// m_TopIntake.ConfigPeakCurrentDuration(1750);
	// m_TopIntake.ConfigPeakCurrentLimit(6);
	// m_TopIntake.ConfigContinuousCurrentLimit(2);
	// m_TopIntake.EnableCurrentLimit(true);
	// m_BottomIntake.ConfigPeakCurrentDuration(1750);
	// m_BottomIntake.ConfigPeakCurrentLimit(7);
	// m_BottomIntake.ConfigContinuousCurrentLimit(3.5);
	// m_BottomIntake.EnableCurrentLimit(true);
 

	//creating configuration profile (could be more efficient?) - LOOK, before it was just one line and now its three lines 
	configs::MagnetSensorConfigs canConfigs{};
	m_PivotCANCoder.GetConfigurator().Apply(canConfigs.WithAbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Unsigned_0To1));

	//are the raw sensor units equal to the "mechanism rotations" - DEFINITELY LOOK, not completely sure about the units here
	m_Pivot.SetPosition(units::angle::turn_t(CANCODER_ZERO - m_PivotCANCoder.GetAbsolutePosition().GetValueAsDouble()) /* * PIVOT_TICKS_PER_DEGREE*/);
	
	// m_Pivot.SetSelectedSensorPosition((CANCODER_ZERO - m_PivotCANCoder.GetAbsolutePosition()) * PIVOT_TICKS_PER_DEGREE);
	// m_Wrist.SetSelectedSensorPosition((WristStringPotUnitsToTicks(m_StringPot.GetValue())));
}

void Arm::SetButtons()
{
	m_Override.OnTrue(ManualControls());

	m_IntakeButton.OnTrue(DynamicIntake().ToPtr());
	m_OuttakeButton.OnTrue(DynamicIntake().ToPtr());

	m_GroundPickupMode.OnTrue(new frc2::InstantCommand([&]{
		Robot::GetRobot()->GetArm().m_PivotPos = 98.0;
      	Robot::GetRobot()->GetArm().m_WristPos = 3.0;
		SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
		new frc2::ParallelCommandGroup(
			frc2::PrintCommand("Ground Pickup"),
			PivotToPos(), 
      		WristToPos()
	  	);
	}));

	m_TransitMode.OnTrue(new frc2::InstantCommand([&]{
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
		m_Pivot.SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(units::voltage::volt_t(Robot::GetRobot()->GetButtonBoard().GetRawAxis(PIVOT_CONTROL) / 2 * 12.0)));
		m_Wrist.SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(units::voltage::volt_t(Robot::GetRobot()->GetButtonBoard().GetRawAxis(WRIST_CONTROL) / 2 * 12.0)));

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
	
	//from percent output to voltage out ): - DEFINITELY LOOK, changed the if statements to just change the value of the voltage,
	//and set the control statement outside of the if-statement just to make it a little neater
	m_BottomIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);


	if(Robot::GetRobot()->GetButtonBoard().GetRawButton(INTAKE_BUTTON)) m_BottomIntakeVoltage = 12 * power; /*m_BottomIntake.Set(ControlMode::PercentOutput, power);*/
	else if (Robot::GetRobot()->GetButtonBoard().GetRawButton(OUTTAKE_BUTTON)) m_BottomIntakeVoltage = 12; /*m_BottomIntake.SetControl(m_request.WithOutput(12_V));*/
	else m_BottomIntakeVoltage = 0;

	},[&](bool e) { // onEnd
		m_Pivot.SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(0_V));
		m_Wrist.SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(0_V));
		m_BottomIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
	},
	[&] { // isFinished
		return !Robot::GetRobot()->GetButtonBoard().GetRawButton(ARM_OVERRIDE);
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
	m_Pivot.GetConfigurator().Apply(pivotMotionMagicConfigs, 0_s);
	m_Wrist.GetConfigurator().Apply(wristMotionMagicConfigs, 0_s);
}