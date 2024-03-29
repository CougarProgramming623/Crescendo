#include "commands/BotStrafe.h"
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
using ctre::phoenix::motorcontrol::ControlMode;

# define DRIVE Robot::GetRobot()->GetDriveTrain()

# define ROBOT Robot::GetRobot()

# define ARM Robot::GetRobot()->GetArm()

BotStrafe::BotStrafe() {}

void BotStrafe::Initialize() {

}

void BotStrafe::Execute() {
    
}

void BotStrafe::End(bool interrupted){
}

bool BotStrafe::IsFinished(){
    return false;
}