#include "commands/Intake.h"
#include "Robot.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <frc2/command/button/Trigger.h>
#include <frc/AnalogInput.h>
#include <math.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/motorcontrol/NeutralMode.h>

#include "Constants.h"
#include "Util.h"
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <subsystems/Drivetrain.h>


using namespace ctre::phoenix6;
using namespace ctre::phoenix;
using ctre::phoenix::motorcontrol::NeutralMode;



Intake::Intake()
        //m_BigRed(frc2::Trigger(BUTTON_L(BIG_RED)))
    {
    }
/*
initialize values
*/
void Intake::Initialize() {
    //Motors are initialized in drivetrain because the drivetrain class is not called multiple times so there is no need for a default constructor
    //Robot::GetRobot()->GetDriveTrain().m_IntakeMotor1.SetNeutralMode(NeutralMode::Brake);
    //Robot::GetRobot()->GetDriveTrain().m_DustpanLaunch.SetNeutralMode(NeutralMode::Brake);
    //Robot::GetRobot()->GetDriveTrain().m_IntakeMotor2.SetInverted(true);
    //power1 = 0.2;
    //power2 = 0.2;
    //DebugOutF("Initialized");
    //balanced = false;
}
/*
pushes the balanced status and the pitch to the network tables and utilizes PID and the drivetrain BaseDrive()
function to perform the autobalance command
*/
void Intake::Execute() {
     	    Robot::GetRobot()->GetArm().m_ServoShoot.OnTrue(new frc2::InstantCommand([&]{
                    set = 0;
                    Robot::GetRobot()->GetDriveTrain().m_DustpanLaunch.Set(set);
                    frc2::WaitCommand(1_s);
                    DebugOutF("Wow the code works1" + std::to_string(set));
                    set = 1;
                    Robot::GetRobot()->GetDriveTrain().m_DustpanLaunch.Set(set);
                   DebugOutF("Wow the code works 2" + std::to_string(set));
                })
         
            );

    //set = Robot::GetRobot()->GetButtonBoard().GetRawAxis(1);
    //Robot::GetRobot()->GetDriveTrain().m_DustpanLaunch.SetAngle(180);
    // Robot::GetRobot()->GetDriveTrain().m_IntakeMotor1.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(power1));
    // Robot::GetRobot()->GetDriveTrain().m_IntakeMotor2.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(power2));

    ///Robot::GetRobot()->m_MR.OnTrue(
        //new frc2::InstantCommand([&]{
            //power2 = 0;
        //})
    //);

    // Robot::GetRobot()->m_RightGrid.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power2 *= -1;
    //     })
    // );

    // Robot::GetRobot()->m_LeftGrid.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power1 *= -1;
    //     })
    // );

    // Robot::GetRobot()->m_TL.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power1 += 0.5;
    //     })
    // );

    // Robot::GetRobot()->m_TR.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power2 += 0.5;
    //     })
    // );

    //  Robot::GetRobot()->m_ML.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power1 = 0;
    //     })
    // );

    // Robot::GetRobot()->m_MR.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power2 = 0;
    //     })
    // );

    //  Robot::GetRobot()->m_BL.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power1 -= 0.5;
    //     })
    // );

    // Robot::GetRobot()->m_BR.OnTrue(
    //     new frc2::InstantCommand([&]{
    //         power2 -= 0.5;
    //     })
    // );

    // double angle = Robot::GetRobot()->GetNavX().GetPitch() + 0.05;
    // if(angle > -15 && angle < 15) (balanced = true);
    // Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/balanced").SetBoolean(balanced);

    // double kPy = 0.01;//0.00005;
    // double kIy = 0;//0.0000001;
    // double kDy = 0;//0.0001;

    // double kPx = 0.01;
    // double kIx = 0;
    
    // double kPt = 0.01;
    // double kIt = 0;

    // double m_currentAngleY = Robot::GetRobot()->GetNavX().GetPitch() + 0.5;
    // double m_currentAngleX = Robot::GetRobot()->GetNavX().GetRoll() + 0.5;
    // double m_currentAngleT = Robot::GetRobot()->GetAngle(); 

    // double errorX = 0 /*setpoint constant*/ - m_currentAngleX;
    // double errorY = 0 /*setpoint constant*/ - m_currentAngleY;
    // double errorT = 0 /*setpoint constant*/ - m_currentAngleT;

    // if(errorY < 0.5 && errorY > -0.5) {
    //     Robot::GetRobot()->previousErrorY = 0;
    //     Robot::GetRobot()->previousValueY = 0;
    // }

    // //double outputX = Robot::GetRobot()->previousValueX + (kPx * errorX) + (kIx * (Robot::GetRobot()->previousErrorX));
    // //double outputY = Robot::GetRobot()->previousValueY + (kPy * errorY) + (kIy * (Robot::GetRobot()->previousErrorY)) + (kDy * (errorY - Robot::GetRobot()->dErrorY) / (0.02));
    // //double outputY = (kPy * errorY) + (kDy * (errorY - Robot::GetRobot()->previousErrorY) / (0.02));
    // double outputY = (kPy * errorY);
    // double outputX = (kPx * errorX);
    // double outputT = (kPt * errorT);
    // //double outputT = Robot::GetRobot()->previousValueT + (kPt * errorT) + (kIt * (Robot::GetRobot()->previousErrorT));

    // Robot::GetRobot()->previousErrorX += errorX;
    // Robot::GetRobot()->previousValueX = outputX;
    // Robot::GetRobot()->previousErrorY += errorY;
    // Robot::GetRobot()->previousValueY = outputY;
    // Robot::GetRobot()->previousErrorT += errorT;
    // Robot::GetRobot()->previousValueT = outputT;

    // //double output = Y[k-1] + kP * U[k] + kI * U[k-1]  
    // //  current value = previous value + kP * currentError + kI * previousError

    // outputX = (std::abs(outputX) > 1) ? 1 : outputX;
    // outputY = (std::abs(outputY) > 1) ? 1 : outputY;
    // outputT = (std::abs(outputT) > 1) ? 1 : outputT;

    // outputY = (m_currentAngleT >= 270 && m_currentAngleT <= 90) ? outputY : -outputY;
    // outputX = (m_currentAngleT >= 315 && m_currentAngleT <= 45) ? -outputX : outputX;


    // Robot *r = Robot::GetRobot();
    
    // r->GetDriveTrain().BaseDrive(
    //     frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //         //units::meters_per_second_t(0 * outputX * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //x
    //         units::meters_per_second_t(outputY * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //y
    //         units::meters_per_second_t(-outputX * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //x
    //         units::radians_per_second_t(/*outputT*/0 * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND), //rotation
    //         frc::Rotation2d(units::radian_t(/*Deg2Rad(-fmod(360 - r->GetNavX().GetAngle(), 360)*/0))
    //     )
    // );
    // // DebugOutF("Error: " + std::to_string(errorY));
    // // DebugOutF("Output: " + std::to_string(outputY));
    // //DebugOutF(std::to_string(m_currentAngleT));
    // Robot::GetRobot()->dErrorY = errorY;
}

void Intake::End(bool interrupted){
    // Robot::GetRobot()->GetDriveTrain().m_IntakeMotor1.SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(0_V));
    // Robot::GetRobot()->GetDriveTrain().m_IntakeMotor2.SetControl(Robot::GetRobot()->m_VoltageOutRequest.WithOutput(0_V));
}

bool Intake::IsFinished(){
    return false; //Robot::GetRobot()->GetJoyStick().GetRawAxis(1) > 0.3 || Robot::GetRobot()->GetJoyStick().GetRawAxis(0) > 0.3 || Robot::GetRobot()->GetJoyStick().GetRawAxis(2) > 0.3;
}