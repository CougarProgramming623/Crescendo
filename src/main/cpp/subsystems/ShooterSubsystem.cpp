// #include <frc/TimedRobot.h>
// #include "ctre/Phoenix.h"
// #include <frc/Joystick.h>

// class Robot : public frc::TimedRobot {
// public:
//     void RobotInit() {
//         shooterWheel1 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
//         shooterWheel2 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
//         dustpanAngleMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
//         feederMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(3);
//         shooterAngleMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(4);

//         joystick = new frc::Joystick(0);
//     }

//     void TeleopPeriodic() {
//         if (joystick->GetRawButton(1)) {
//             shooterWheel1->Set(1.0);
//             shooterWheel2->Set(1.0);
//         } else if (joystick->GetRawButton(2)) {
//             shooterWheel1->Set(-1.0);
//             shooterWheel2->Set(-1.0);
//         } else {
//             shooterWheel1->Set(0.0);
//             shooterWheel2->Set(0.0);
//         }

//         if (joystick->GetRawButton(3)) {
//             dustpanAngleMotor->Set(1.0);
//         } else if (joystick->GetRawButton(4)) {
//             dustpanAngleMotor->Set(-1.0);
//         } else {
//             dustpanAngleMotor->Set(0.0);
//         }

//         if (joystick->GetRawButton(5)) {
//             feederMotor->Set(1.0);
//         } else {
//             feederMotor->Set(0.0);
//         }

//         if (joystick->GetRawButton(6)) {
//             shooterAngleMotor->Set(1.0);
//         } else if (joystick->GetRawButton(7)) {
//             shooterAngleMotor->Set(-1.0);
//         } else {
//             shooterAngleMotor->Set(0.0);
//         }
//     }

// private:
//     ctre::phoenix::motorcontrol::can::WPI_TalonFX* shooterWheel1;
//     ctre::phoenix::motorcontrol::can::WPI_TalonFX* shooterWheel2;
//     ctre::phoenix::motorcontrol::can::WPI_TalonFX* dustpanAngleMotor;
//     ctre::phoenix::motorcontrol::can::WPI_TalonFX* feederMotor;
//     ctre::phoenix::motorcontrol::can::WPI_TalonFX* shooterAngleMotor;
//     frc::Joystick* joystick;
// };

// #ifndef RUNNING_FRC_TESTS
// int main() { return frc::StartRobot<Robot>(); }
// #endif

// cant find header file for some reason so this is temporarily commented out

// #include "ShooterSubsystem.h"

// ShooterSubsystem::ShooterSubsystem() {
//     shooterWheel1 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
//     shooterWheel2 = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
//     dustpanAngleMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
//     feederMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(3);
//     shooterAngleMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(4);

//     joystick = new frc::Joystick(0);
// }

// void ShooterSubsystem::TeleopPeriodic() {
//     if (joystick->GetRawButton(1)) {
//         shooterWheel1->Set(1.0);
//         shooterWheel2->Set(1.0);
//     } else if (joystick->GetRawButton(2)) {
//         shooterWheel1->Set(-1.0);
//         shooterWheel2->Set(-1.0);
//     } else {
//         shooterWheel1->Set(0.0);
//         shooterWheel2->Set(0.0);
//     }

//     if (joystick->GetRawButton(3)) {
//         dustpanAngleMotor->Set(1.0);
//     } else if (joystick->GetRawButton(4)) {
//         dustpanAngleMotor->Set(-1.0);
//     } else {
//         dustpanAngleMotor->Set(0.0);
//     }

//     if (joystick->GetRawButton(5)) {
//         feederMotor->Set(1.0);
//     } else {
//         feederMotor->Set(0.0);
//     }

//     if (joystick->GetRawButton(6)) {
//         shooterAngleMotor->Set(1.0);
//     } else if (joystick->GetRawButton(7)) {
//         shooterAngleMotor->Set(-1.0);
//     } else {
//         shooterAngleMotor->Set(0.0);
//     }
// }

