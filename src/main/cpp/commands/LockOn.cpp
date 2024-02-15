#include "commands/LockOn.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "./subsystems/Drivetrain.h"

LockOn::LockOn() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void LockOn::Initialize(){}

void LockOn::Execute() {    
    Robot* r = Robot::GetRobot();
    if(Robot::GetRobot()->GetVision().GetLimeLight()->GetNumber("<tv>",0.0) ==  1) 
        m_GoalTheta = Robot::GetRobot()->GetVision().VisionRobotYaw(Robot::GetRobot()->GetVision().GetFieldPose(), (Robot::GetRobot()->GetVision().GetLimeLight()->GetNumberArray("<tid>",std::vector<double>(6))[0]));
    DebugOutF(std::to_string(m_GoalTheta.Degrees().value()));
    DebugOutF("Act: " + std::to_string(r->GetAngle()));
    DebugOutF(std::to_string(fmod(360 + 90 - r->GetNavX().GetAngle(), 360)));
    

    //speeds.omega = -speeds.omega;
    
    //r->GetDriveTrain().BaseDrive(speeds);
}