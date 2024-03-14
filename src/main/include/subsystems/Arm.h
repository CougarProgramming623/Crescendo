#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include <frc/AnalogInput.h>
#include <math.h>
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
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "./commands/PivotToPos.h"

using namespace ctre::phoenix6;
using namespace ctre::phoenix;

class Arm : public frc2::SubsystemBase {

	public:
	Arm();
	void Init();
	void SetButtons();
	

	frc2::FunctionalCommand* ManualControls();
	void SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc);

	// inline double PivotStringPotUnitsToDegrees(double units) {return ((units - STRINGPOT_ZERO) * PIVOT_DEGREES_PER_STRINGPOT_UNITS + STRINGPOT_ZERO_DEGREES); }
	// inline double PivotStringPotUnitsToRotations(double units) {return PivotDegreesToRotations(PivotStringPotUnitsToDegrees(units));}
	inline double PivotStringPotUnitsToRotations(double units) {return 0;}

	// inline double PivotDegreesToStringPotUnits(double degrees) {return ((degrees / PIVOT_DEGREES_PER_STRINGPOT_UNITS) + STRINGPOT_ZERO); }
	// inline double PivotRotationsToStringPotUnits(double rotations) {return PivotDegreesToStringPotUnits(PivotRotationsToDegrees(rotations));}
	
	inline double PivotDegreesToRotations(double degrees) {return degrees/PIVOT_TOTAL_DEGREES / 360;}
	inline double PivotRotationsToDegrees(double rotations) {return rotations/PIVOT_TOTAL_ROTATIONS * 360 + STRINGPOT_ZERO_DEGREES;}

	
	//getters
	inline hardware::TalonFX& GetPivotMotor() {return m_Pivot;}
	inline hardware::TalonFX& GetClimbMotor() {return m_Climb;}
	inline hardware::TalonFX& GetShooterMotor1() {return m_ShooterMotor1;}
	inline hardware::TalonFX& GetShooterMotor2() {return m_ShooterMotor2;}
	inline motorcontrol::can::TalonSRX& GetFeeder() {return m_Feeder;}
	inline frc2::Trigger& GetArmOverrideButton() {return m_ArmOverride; }
	inline frc::AnalogInput& GetStringPot() {return m_StringPot;}

	frc2::Trigger m_PlacingMode;

	double m_WristPos;
	double m_PivotPos;
	double m_FlywheelPower;
	double m_FlywheelPowerLock;

	double m_OriginalPivotRotations;
	double m_StringPotOffset;

	private:
	
	//motors
	hardware::TalonFX m_Pivot;
	hardware::TalonFX m_Climb;
	hardware::TalonFX m_ShooterMotor1;
	hardware::TalonFX m_ShooterMotor2;
	motorcontrol::can::TalonSRX m_Feeder;

	//potentiometer
	frc::AnalogInput m_StringPot{4};

	//triggers
	frc2::Trigger m_ArmOverride;
	frc2::Trigger m_ShooterUp;
	frc2::Trigger m_ShooterDown;
	frc2::Trigger m_FlywheelPowerLock;
	frc2::Trigger m_RunFlywheel;
	

	// frc2::Trigger m_ConeMode;
	// frc2::Trigger m_CubeMode;

	// frc2::Trigger m_IntakeButton;
	// frc2::Trigger m_OuttakeButton;

	frc::Timer m_Timer;

	frc2::SequentialCommandGroup* m_Top;
	frc2::SequentialCommandGroup* m_Mid;
	frc2::SequentialCommandGroup* m_Bot;
};