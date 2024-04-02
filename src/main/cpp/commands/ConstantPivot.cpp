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

# define r Robot::GetRobot()

ConstantPivot::ConstantPivot() {
    AddRequirements(&Robot::GetRobot()->GetArm());
}

void ConstantPivot::Initialize() {}

void ConstantPivot::Execute() {
    stringpot = r->GetArm().GetStringPot().GetAverageValue();
    Vision vision = r->GetVision();
    // DebugOutF("Stringpot value before the if statement" + std::to_string(r->GetArm().GetStringPot().GetAverageValue()));
    if(stringpot < STRINGPOT_LOW){
        PivotToPos(stringpot + 5);
    }
    if(vision.GetLimeLight()->GetNumber("tv", 0.0) == 1) {
        int id = vision.GetLimeLight()->GetNumber("tid", 0.0);
        double distance = vision.DistanceFromAprilTag(id);
        int val = (r->GetArm().DistanceToStringPotUnits(distance));
        stringpot = r->GetArm().GetStringPot().GetAverageValue();
        //DebugOutF("stringpot average value" + std::to_string(stringpot));
        int difference = stringpot - val;
        //DebugOutF("difference value : " + std::to_string(difference));
            DebugOutF("target: " + std::to_string(val));
        if(abs(difference) < 2) {
            DebugOutF("Setting motor to 0");
            r->GetArm().GetPivotMotor().Set(0);
        }
        if(distance <= 3.26 && val > STRINGPOT_LOW && val < 555) {
            //DebugOutF("Average value of stringpot close up" + std::to_string(r->GetArm().GetStringPot().GetAverageValue()));
            double kp = 0.1;
            double targetSpeed = kp * difference;
            r->GetArm().GetPivotMotor().Set(targetSpeed);
        }
    } else {r->GetArm().GetPivotMotor().Set(0);}
    // else if(r->GetArm().GetStringPot().GetAverageValue() > STRINGPOT_LOW) {
    //     stringpot = r->GetArm().GetStringPot().GetAverageValue();
    //     int difference = stringpot - 420;
    //     double kp = 0.125;
    //     double targetSpeed = kp * difference;
    //     r->GetArm().GetPivotMotor().Set(targetSpeed);
    //     r->GetArm().GetDustpanPivotServo().Set(1);
    // }
}

void ConstantPivot::End(bool interrupted) {
    r->GetArm().GetPivotMotor().Set(0);
}

bool ConstantPivot::IsFinished() {
    return !r->GetArm().GetAimButton().Get();
}