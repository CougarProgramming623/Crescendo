#include "ctre/Phoenix.h"
#include <frc/Joystick.h>

class Shooter {
public:
    Shooter() {
        flywheel1 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
        flywheel2 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
    }

    void Operate(frc::Joystick* joystick) {
        if (joystick->GetRawButton(1)) {
            flywheel1->Set(1.0);
            flywheel2->Set(1.0);
        } else {
            flywheel1->Set(0.0);
            flywheel2->Set(0.0);
        }
    }

private:
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* flywheel1;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX* flywheel2;
};
