#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include <frc/AnalogInput.h>
#include <math.h>
#//include "Robot.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
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
	void ArmInit();
	void Periodic() override;
	void SetButtons();
	// void MoveToStringPotValue(int target);
	//int ConvertDistanceToValue();

	frc::Servo m_DustpanLaunch {0};

	frc2::FunctionalCommand* ManualControls();

	// void SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc);
	// inline double PivotDegreesToStringPotUnits(double degrees) {return ((degrees / PIVOT_DEGREES_PER_STRINGPOT_UNITS) + STRINGPOT_ZERO); }
	// inline double PivotRotationsToStringPotUnits(double rotations) {return PivotDegreesToStringPotUnits(PivotRotationsToDegrees(rotations));}
	//inline double PivotDegreesToRotations(double degrees) {return degrees/PIVOT_TOTAL_DEGREES / 360;}
	//inline double PivotRotationsToDegrees(double rotations) {return rotations/PIVOT_TOTAL_ROTATIONS * 360 + STRINGPOT_ZERO_DEGREES;}
	inline double PivotDegreesToStringPotLength(double degrees) {return sqrt((ARM_LENGTH * ARM_LENGTH) + (DIFF_BASE_PIVOT_STRINGPOT * DIFF_BASE_PIVOT_STRINGPOT) - (2 * DIFF_BASE_PIVOT_STRINGPOT * ARM_LENGTH * cos(degrees)));}

	//Need to get the ratio of units to length ASAP
	inline int DistanceToStringPotUnits(double distance) {return (a * pow(distance, 8) + b * pow(distance, 7) + c * pow(distance, 6) + d * pow(distance, 5) + f * pow(distance, 4) +  g * pow(distance, 3) +  h * pow(distance, 2) +  i * pow(distance, 1) + j);}
	inline double StringPotUnitsToVelocity(double units) {return (-1);}
	inline int StringPotLengthToStringPotUnits(double len) {return -1;}
	inline double StringPotUnitsToRotations(int val) {return PIVOT_LOW  - (((val - STRINGPOT_LOW)/STRINGPOT_TOTAL_RANGE) * PIVOT_TOTAL_ROTATIONS);}
	
	//getters
	inline hardware::TalonFX& GetPivotMotor() {return m_Pivot;}
	inline hardware::TalonFX& GetShooterMotor1() {return m_ShooterMotor1;}
	inline hardware::TalonFX& GetShooterMotor2() {return m_ShooterMotor2;}
	inline motorcontrol::can::TalonSRX& GetFeeder() {return m_Feeder;}
	inline frc::Servo& GetDustpanLaunchServo() {return m_DustpanLaunch;}
	inline frc::Servo& GetDustpanPivotServo() {return m_DustpanPivot;}
	inline frc::AnalogInput& GetStringPot() {return m_StringPot;}
	inline frc2::Trigger& GetShooterUpButton() {return m_ShooterUp;}
	inline frc2::Trigger& GetShooterDownButton() {return m_ShooterDown;}
	inline frc2::Trigger& GetAimButton() {return m_Aim;}
	

	frc2::Trigger m_PlacingMode;

	double m_FlywheelPower = 0.88;
	double m_Differential = 0.8;
	units::turns_per_second_t m_FlywheelVelocity = units::turns_per_second_t(72);

	controls::VelocityDutyCycle m_VelocityDutyCycle{units::turns_per_second_t(0)};

	double m_OriginalPivotRotations;
	double m_StringPotOffset;
	int m_StringPotValue;

	private:

	double a = 98.7315;
	double b = -1789.18;
	double c = 13816.4;
	double d = -59215.1;
	double f = 153564;
	double g = -245904;
	double h = 236676;
	double i = -125018;
	double j = 28301.5;


	//triggers
	frc2::Trigger m_ShooterUp;
	frc2::Trigger m_ShooterDown;
	frc2::Trigger m_FlywheelPowerLock;
	frc2::Trigger m_RunFlywheel;
	frc2::Trigger m_DustpanUp;
  	frc2::Trigger m_DustpanDown;
	frc2::Trigger m_IntakeSwitch;
	frc2::Trigger m_ServoShoot;
	frc2::Trigger m_Aim;
	frc2::Trigger m_PickupPivot;
	frc2::Trigger m_ProtectedBlockPivot;
	frc2::Trigger m_CloseShootPivot;
	frc2::Trigger m_ChangeDifferential;
	
	//motors
	
	hardware::TalonFX m_ShooterMotor1;
	hardware::TalonFX m_ShooterMotor2;
	hardware::TalonFX m_Pivot;
	motorcontrol::can::TalonSRX m_Feeder;
	//servos
  	frc::Servo m_DustpanPivot {1};

	//potentiometer
	frc::AnalogInput m_StringPot{4};

	frc::Timer m_Timer;
};