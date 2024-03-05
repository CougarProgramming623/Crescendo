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


#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/PrintCommand.h>
//#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
#include "Util.h"
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/ParallelCommandGroup.h>
//#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "./commands/PivotToPos.h"
#include "./commands/DynamicIntake.h"
//#include "./commands/WristToPos.h"

using namespace ctre::phoenix6;
//using ctre::phoenix::motorcontrol::can::TalonSRX;
//using ctre::phoenix6::


class Arm : public frc2::SubsystemBase {

	public:
	Arm();
	void Init();
	void SetButtons();
	frc2::Trigger m_ServoShoot;
	frc2::Trigger m_OuttakeButton;
	

	frc2::FunctionalCommand* ManualControls();
	void SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc);

	inline double WristStringPotUnitsToDegrees(double units) {return -((units - STRINGPOT_ZERO) * WRIST_DEGREES_PER_STRINGPOT_UNITS); }
	inline double WristDegreesToStringPotUnits(double degrees) {return -((degrees / WRIST_DEGREES_PER_STRINGPOT_UNITS) + STRINGPOT_ZERO); }
	
	inline double WristStringPotUnitsToTicks(double units) {return WristDegreesToTicks(WristStringPotUnitsToDegrees(units));}
	inline double WristTicksToStringPotUnits(double ticks) {return WristDegreesToStringPotUnits(WristTicksToDegrees(ticks));}
	inline double WristDegreesToTicks(double degrees) {return degrees * WRIST_TICKS_PER_DEGREE;}
	inline double WristTicksToDegrees(double ticks) {return ticks / WRIST_TICKS_PER_DEGREE;}

	inline double PivotDegreesToTicks(double degrees) {return degrees * PIVOT_TICKS_PER_DEGREE;}
	inline double PivotTicksToDegrees(double ticks) {return ticks / PIVOT_TICKS_PER_DEGREE;}

	
	//getters
	//inline hardware::TalonFX& GetPivotMotor() {return m_Pivot;}
	//inline hardware::TalonFX& GetWristMotor() {return m_Wrist;} 
	// inline TalonSRX& GetTopIntakeMotor() {return m_TopIntake;}
	//inline hardware::TalonFX& GetBottomIntakeMotor() {return m_BottomIntake;}
	// inline rev::CANSparkMax& GetBottomIntakeMotor() {return m_BottomIntake;}
	//inline hardware::CANcoder& GetPivotCANCoder() {return m_PivotCANCoder;}
	inline frc2::Trigger& GetCubeModeButton() {return m_ShooterDown; }
	inline frc2::Trigger& GetConeModeButton() {return m_ShooterUp; }
	inline frc2::Trigger& GetIntakeButton() {return m_ServoShoot; }
	inline frc2::Trigger& GetOuttakeButton() {return m_OuttakeButton; }
	inline frc::AnalogInput& GetStringPot() {return m_StringPot;}

	//Make sure to use this when recording values for the setpoints
	double m_PivotMatrix[3][3] = {
		{-8.0, -33.0, 25.0},
		{-20.0, 58.5, -20.0},
		{50, 50, 50},
	};
	//Idk if we need this anymore probably not
	double m_WristMatrix[3][3] = {
		{28, 46.0, -121.0},
		{30.0, 60, 30.0},
		{-40, -40, -40},
	};
	frc2::Trigger m_PlacingMode;

	//double m_WristPos;
	double m_PivotPos;

	private:
	
	//motors
	//hardware::TalonFX m_Pivot;
	//hardware::CANcoder m_PivotCANCoder{PIVOT_CAN_ID};
	//hardware::TalonFX m_Wrist;
	//hardware::TalonFX m_BottomIntake;
	// rev::CANSparkMax m_BottomIntake;
	// TalonSRX m_TopIntake;


	//motor control voltages
	int m_BottomIntakeVoltage;
	// units::voltage::volt_t m_PivotVoltage;
	// units::voltage::volt_t m_WristVoltage;

	//Stringpot initalization will probably use this to measure angle or current shooter
	frc::AnalogInput m_StringPot{STRINGPOT};

	//triggers
	frc2::Trigger m_StowButton;
	frc2::Trigger m_AmpButton;

	frc2::Trigger m_Override;
	frc2::Trigger m_Override2;

	frc2::Trigger m_ShooterUp;
	frc2::Trigger m_ShooterDown;


	frc::Timer m_Timer;


	frc2::SequentialCommandGroup* m_Top;
	frc2::SequentialCommandGroup* m_Mid;
	frc2::SequentialCommandGroup* m_Bot;
};