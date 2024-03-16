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
	// void SetMotionMagicValues(double pivotVel, double pivotAcc, double wristVel, double wristAcc);
	inline double PivotStringPotUnitsToRotations(double units) {return 0;}
	
	inline double PivotDegreesToRotations(double degrees) {return degrees/PIVOT_TOTAL_DEGREES / 360;}
	inline double PivotRotationsToDegrees(double rotations) {return rotations/PIVOT_TOTAL_ROTATIONS * 360 + STRINGPOT_ZERO_DEGREES;}

	
	//getters
	inline hardware::TalonFX& GetPivotMotor() {return m_Pivot;}
	inline hardware::TalonFX& GetClimbMotor() {return m_Climb;}
	inline hardware::TalonFX& GetShooterMotor1() {return m_ShooterMotor1;}
	inline hardware::TalonFX& GetShooterMotor2() {return m_ShooterMotor2;}
	inline motorcontrol::can::TalonSRX& GetFeeder() {return m_Feeder;}
	inline frc::AnalogInput& GetStringPot() {return m_StringPot;}
	inline frc2::Trigger& GetDustpanUp() {return m_DustpanUp; }
	inline frc2::Trigger& GetDustpanDown() {return m_DustpanDown; }
	inline frc2::Trigger& GetClimbDown() {return m_ClimbDown; }
	inline frc2::Trigger& GetClimbUp() {return m_ClimbUp; }

	frc2::Trigger m_PlacingMode;

	double m_FlywheelPower;

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
	frc2::Trigger m_DustpanUp;
  	frc2::Trigger m_DustpanDown;
	frc2::Trigger m_ClimbUp;
	frc2::Trigger m_ClimbDown;

	frc::Timer m_Timer;
};