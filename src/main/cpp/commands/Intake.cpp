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

# define DRIVE Robot::GetRobot()->GetDriveTrain()

# define ROBOT Robot::GetRobot()

# define ARM Robot::GetRobot()->GetArm()

 Intake::Intake()
     {}
void Intake::Initialize() {
}
void Intake::Execute() {
    	ARM.GetDustpanUp().OnTrue(new frc2::InstantCommand([&]{
            DRIVE.m_DustpanRotate.Set(0);
            frc2::WaitCommand(1_s);
            DRIVE.m_DustpanLaunch.Set(0.75);
	    }));
        ARM.GetDustpanDown().OnTrue(new frc2::InstantCommand([&]{
            DRIVE.m_DustpanRotate.Set(1);
            frc2::WaitCommand(1_s);
            DRIVE.m_DustpanLaunch.Set(1);
	    }));
}

void Intake::End(bool interrupted){
}

bool Intake::IsFinished(){
    return false;
}