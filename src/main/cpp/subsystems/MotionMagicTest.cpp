#include "subsystems/MotionMagicTest.h"

using ctre::phoenix::motorcontrol::ControlMode;

MotionMagicTest::MotionMagicTest() : 
	m_TestMotor(30)
{
	//P = 0.00899982452
	//I = 1.90734863E-06
	//D = 0.25
	
	//WRIST
	// m_TestMotor.Config_kF(0, 0.06089285714, 0);
	// // m_TestMotor.Config_kP(0, 0.006730263158, 0);
	// m_TestMotor.ConfigMotionCruiseVelocity(8400, 0);
	// m_TestMotor.ConfigMotionAcceleration(16800, 0);

	//PIVOT
	// m_TestMotor.SetInverted(true);
	m_TestMotor.Config_kF(0, 0.0639375, 0);
	m_TestMotor.ConfigMotionCruiseVelocity(8400, 0);
	m_TestMotor.ConfigMotionAcceleration(16800, 0);
}

void MotionMagicTest::MotionMagicTestInit() {
	// m_TestMotor.Set(ControlMode::MotionMagic, 20000);
}