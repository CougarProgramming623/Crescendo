#include "commands/AutoLock.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "./subsystems/Drivetrain.h"

AutoLock::AutoLock() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void AutoLock::Initialize(){}