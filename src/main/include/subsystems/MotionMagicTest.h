#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

using ctre::phoenix::motorcontrol::can::TalonSRX;

class MotionMagicTest {
	public:
		MotionMagicTest();
		void MotionMagicTestInit();

	private:

	TalonSRX m_TestMotor;
};
