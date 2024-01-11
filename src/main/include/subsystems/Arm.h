#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <ctre/phoenix/motorcontrol/can/BaseMotorController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include <frc/AnalogInput.h>
#include <math.h>
#include <ctre/phoenix6/CANCoder.hpp>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>


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
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "./commands/PivotToPos.h"
#include "./commands/DynamicIntake.h"
#include "./commands/WristToPos.h"

using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix::motorcontrol::can::TalonSRX;


class Arm : public frc2::SubsystemBase {

	public:

	Arm();
	void Init();
	void SetButtons();
	

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
	inline TalonSRX& GetPivotMotor() {return m_Pivot;}
	inline TalonSRX& GetWristMotor() {return m_Wrist;} 
	// inline TalonSRX& GetTopIntakeMotor() {return m_TopIntake;}
	inline TalonSRX& GetBottomIntakeMotor() {return m_BottomIntake;}
	// inline rev::CANSparkMax& GetBottomIntakeMotor() {return m_BottomIntake;}
	inline ctre::phoenix::sensors::CANCoder& GetPivotCANCoder() {return m_PivotCANCoder;}
	inline frc2::JoystickButton& GetCubeModeButton() {return m_CubeMode; }
	inline frc2::JoystickButton& GetConeModeButton() {return m_ConeMode; }
	inline frc2::JoystickButton& GetIntakeButton() {return m_IntakeButton; }
	inline frc2::JoystickButton& GetOuttakeButton() {return m_OuttakeJoystickButton; }
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
	frc2::JoystickButton m_PlacingMode;

	double m_WristPos;
	double m_PivotPos;

	private:
	
	//motors
	TalonSRX m_Pivot; 
	ctre::phoenix::sensors::CANCoder m_PivotCANCoder{PIVOT_CAN_ID};
	TalonSRX m_Wrist; 
	TalonSRX m_BottomIntake;
	// rev::CANSparkMax m_BottomIntake;

	// TalonSRX m_TopIntake;

	//pot
	frc::AnalogInput m_StringPot{STRINGPOT};

	//JoystickButtons
	frc2::JoystickButton m_TransitMode;
	frc2::JoystickButton m_GroundPickupMode;

	frc2::JoystickButton m_Override;
	frc2::JoystickButton m_Override2;

	frc2::JoystickButton m_ConeMode;
	frc2::JoystickButton m_CubeMode;

	frc2::JoystickButton m_IntakeButton;
	frc2::JoystickButton m_OuttakeButton;

	frc::Timer m_Timer;

	frc2::SequentialCommandGroup* m_Top;
	frc2::SequentialCommandGroup* m_Mid;
	frc2::SequentialCommandGroup* m_Bot;
};