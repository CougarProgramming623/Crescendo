#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/Joystick.h>
#include <frc/Servo.h>
//#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include <frc/AnalogInput.h>
#include <math.h>
//#include <ctre/phoenix/sensors/CANCoder.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>


#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
#include "Util.h"
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc/AnalogInput.h>
//#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "./commands/PivotToPos.h"
#include "./commands/DynamicIntake.h"
//#include "./commands/WristToPos.h"

using namespace ctre::phoenix6;
using namespace ctre::phoenix;
//using ctre::phoenix::motorcontrol::can::TalonSRX;
//using ctre::phoenix6::


class Arm : public frc2::SubsystemBase {

	public:
	Arm();
	void Init();
	void SetButtons();
	

	frc2::FunctionalCommand* ManualControls();
	void SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc);

	inline double PivotStringPotUnitsToDegrees(double units) {return ((units - STRINGPOT_ZERO) * PIVOT_DEGREES_PER_STRINGPOT_UNITS + STRINGPOT_ZERO_DEGREES); }
	inline double PivotStringPotUnitsToRotations(double units) {return PivotDegreesToRotations(PivotStringPotUnitsToDegrees(units));}

	inline double PivotDegreesToStringPotUnits(double degrees) {return ((degrees / PIVOT_DEGREES_PER_STRINGPOT_UNITS) + STRINGPOT_ZERO); }
	inline double PivotRotationsToStringPotUnits(double rotations) {return PivotDegreesToStringPotUnits(PivotRotationsToDegrees(rotations));}
	
	inline double PivotDegreesToRotations(double degrees) {return degrees/PIVOT_TOTAL_DEGREES / 360;}
	inline double PivotRotationsToDegrees(double rotations) {return rotations/PIVOT_TOTAL_ROTATIONS * 360 + STRINGPOT_ZERO_DEGREES;}

	
	//getters
	// inline hardware::TalonFX& GetPivotMotor() {return m_Pivot;}
	 inline hardware::TalonFX& GetClimbMotor() {return m_Climb;} 
	// inline TalonSRX& GetTopIntakeMotor() {return m_TopIntake;}
	// inline hardware::TalonFX& GetBottomIntakeMotor() {return m_BottomIntake;}
	// inline rev::CANSparkMax& GetBottomIntakeMotor() {return m_BottomIntake;}
	// inline hardware::CANcoder& GetPivotCANCoder() {return m_PivotCANCoder;}
	   inline frc2::Trigger& GetArmOverrideButton() {return m_ArmOverride; }
	// inline frc2::Trigger& GetConeModeButton() {return m_ConeMode; }
	// inline frc2::Trigger& GetIntakeButton() {return m_IntakeButton; }
	// inline frc2::Trigger& GetOuttakeButton() {return m_OuttakeButton; }
	inline frc::AnalogInput& GetStringPot() {return m_StringPot;}


	double m_PivotMatrix[3][3] = {
		{-8.0, -33.0, 25.0},
		{-20.0, 58.5, -20.0},
		{50, 50, 50},
	};

	double m_WristMatrix[3][3] = {
		{28, 46.0, -121.0},
		{30.0, 60, 30.0},
		{-40, -40, -40},
	};
	frc2::Trigger m_PlacingMode;

	double m_WristPos;
	double m_PivotPos;

	hardware::TalonFX m_ShooterMotor1;
	hardware::TalonFX m_ShooterMotor2;

	private:
	
	//motors
	hardware::TalonFX m_Pivot;
	hardware::TalonFX m_Climb;
	//hardware::CANcoder m_PivotCANCoder{PIVOT_CAN_ID};
	//hardware::TalonFX m_Wrist;
	//motorcontrol::can::TalonSRX m_BottomIntake;
	// rev::CANSparkMax m_BottomIntake;
	// TalonSRX m_TopIntake;


	//motor control voltages
	controls::DutyCycleOut m_DCO{0};
	int m_BottomIntakeVoltage;
	// units::voltage::volt_t m_PivotVoltage;
	// units::voltage::volt_t m_WristVoltage;

	//pot
	frc::AnalogInput m_StringPot{4};


	//triggers
	// frc2::Trigger m_TransitMode;
	// frc2::Trigger m_GroundPickupMode;

	  frc2::Trigger m_ArmOverride;
	// frc2::Trigger m_Override2;
	// frc2::Trigger m_ShooterDown;
	// frc2::Trigger m_ShooterUp;

	// frc2::Trigger m_ConeMode;
	// frc2::Trigger m_CubeMode;

	// frc2::Trigger m_IntakeButton;
	// frc2::Trigger m_OuttakeButton;

	frc::Timer m_Timer;

	frc2::SequentialCommandGroup* m_Top;
	frc2::SequentialCommandGroup* m_Mid;
	frc2::SequentialCommandGroup* m_Bot;
};