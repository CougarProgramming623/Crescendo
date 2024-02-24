// #include "ctre/Phoenix.h"
// #include <frc/Joystick.h>

// class DustPan {
// public:
//     DustPan() {
//         dustPanMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
//     }

//     void Operate(frc::Joystick* joystick) {
//         if (joystick->GetRawButton(2)) {
//             dustPanMotor->Set(1.0);
//         } else if (joystick->GetRawButton(3)) {
//             dustPanMotor->Set(-1.0);
//         } else {
//             dustPanMotor->Set(0.0);
//         }
//     }

// private:
//     ctre::phoenix::motorcontrol::can::WPI_TalonFX* dustPanMotor;
// };
