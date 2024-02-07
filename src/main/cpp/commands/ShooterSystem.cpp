#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/Joystick.h>

class Robot : public frc::TimedRobot {
public:
    void RobotInit() {
        flywheel1 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
        flywheel2 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
        dustPanMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
        feederMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(3);
        angleMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(4);

        // Initialize joystick
        joystick = new frc::Joystick(0);
    }

    void TeleopPeriodic() {
        if (joystick->GetRawButton(1)) {
            flywheel1->Set(1.0);
            flywheel2->Set(1.0);
        } else {
            flywheel1->Set(0.0);
            flywheel2->Set(0.0);
        }

        if (joystick->GetRawButton(2)) {
            dustPanMotor->Set(1.0);
        } else if (joystick->GetRawButton(3)) {
            dustPanMotor->Set(-1.0);
        } else {
            dustPanMotor->Set(0.0);
        }

        if (joystick->GetRawButton(4)) {
            feederMotor->Set(1.0);
        } else {
            feederMotor->Set(0.0);
        }

        if (joystick->GetRawButton(5)) {
            angleMotor->Set(1.0);
        } else if (joystick->GetRawButton(6)) {
            angleMotor->Set(-1.0);
        } else {
            angleMotor->Set(0.0);
        }
    }

private:
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* flywheel1;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* flywheel2;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* dustPanMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* feederMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* angleMotor;
    frc::Joystick* joystick;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
