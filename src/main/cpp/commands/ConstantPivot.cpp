#include "commands/ConstantPivot.h"
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

# define r Robot::GetRobot()

ConstantPivot::ConstantPivot() {}

void ConstantPivot::Initialize() {}

void ConstantPivot::Execute() {
    Vision Flyvision = r->GetVision();
    if(Flyvision.GetLimeLight()->GetNumber("tv", 0.0) == 1) {
        id = Flyvision.GetLimeLight()->GetNumber("tid", 0.0);
        dis = Flyvision.DistanceFromAprilTag(id);
        int val = (r->GetArm().DistanceToStringPotUnits(dis));
        r->GetArm().m_StringPotValue = val;
    }
}

void ConstantPivot::End(bool interrupted) {}

bool ConstantPivot::IsFinished() {
    return false;
}