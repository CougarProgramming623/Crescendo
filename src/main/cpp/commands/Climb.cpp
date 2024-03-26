#include "commands/Climb.h"
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

 Climb::Climb()
     {}
void Climb::Initialize() {
}
void Climb::Execute() {
        // ROBOT->GetClimbUp().OnTrue(new frc2::InstantCommand([&]{
        //     ARM.GetClimbMotor().Set(0.5);
	    // }));
        // ROBOT->GetDustpanDown().OnTrue(new frc2::InstantCommand([&]{
        //     ARM.GetClimbMotor().Set(-0.5);
	    // }));
}

void Climb::End(bool interrupted){
}

bool Climb::IsFinished(){
    return false;
}