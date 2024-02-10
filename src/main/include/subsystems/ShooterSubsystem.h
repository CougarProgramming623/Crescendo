#ifndef SHOOTERSUBSYSTEM_H
#define SHOOTERSUBSYSTEM_H

#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/Joystick.h>

class ShooterSubsystem {
public:
    ShooterSubsystem();
    void TeleopPeriodic();

private:
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* shooterWheel1;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* shooterWheel2;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* dustpanAngleMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* feederMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* shooterAngleMotor;
    frc::Joystick* joystick;
};

#endif
