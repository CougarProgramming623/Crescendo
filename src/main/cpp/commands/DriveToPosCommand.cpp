#include "commands/DriveToPosCommand.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "Util.h"

using namespace pathplanner;


//Constructs a Trajectory Command based on a PathPlannerTrajectory
DriveToPosCommand::DriveToPosCommand() {
    DebugOutF("Constructed");
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

//Start timer
void DriveToPosCommand::Initialize(){
    DebugOutF("Init");
    m_Start = Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition();
    m_End = Robot::GetRobot()->GetDriveTrain().m_TransformedPose;
    Robot::GetRobot()->GetDriveTrain().m_DriveToPoseFlag = true;
    // DebugOutF(std::to_string(Robot::GetRobot()->GetDriveTrain().m_TransformedPose.X().value()));
    DebugOutF("Start: (" + std::to_string(m_Start.Translation().X().value()) + ", " + std::to_string(m_Start.Translation().Y().value()) + ")");
    DebugOutF("End: (" + std::to_string(m_End.Translation().X().value()) + ", " + std::to_string(m_End.Translation().Y().value()) + ")");
    
    vertical = m_End.Y().value() - m_Start.Y().value();
    horizontal = m_End.X().value() - m_Start.X().value();

    m_Trajectory = PathPlanner::generatePath(
        units::meters_per_second_t(2),
		units::meters_per_second_squared_t(0.5),
		false,

        PathPoint(
            m_Start.Translation(), 
            frc::Rotation2d(units::radian_t(atan(vertical/horizontal) + M_PI)),
            m_Start.Rotation()
        ), PathPoint(
            m_End.Translation(), 
            frc::Rotation2d(units::radian_t(atan(vertical/horizontal) + M_PI)), //wrong?
            m_End.Rotation()
        )

        // PathPoint(m_Start.Translation(), frc::Rotation2d(0_deg), frc::Rotation2d(0_deg)), // position, heading(direction of travel), holonomic rotation
        // PathPoint(m_End.Translation(), frc::Rotation2d(45_deg), frc::Rotation2d(0_deg)
    );
    DebugOutF("Path");
    m_Timer.Reset();
    m_Timer.Start();
    DebugOutF("Timer");
}

/*
Calculates required chassis speed to match trajactory
Passes ChassisSpeed object to BaseDrive() function
*/
void DriveToPosCommand::Execute() {
    
    //DebugOutF("Execute");

    Robot* r = Robot::GetRobot();
    
    frc::ChassisSpeeds speeds = r->GetDriveTrain().GetHolonomicController().Calculate(
        r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), 
        m_Trajectory.sample(m_Timer.Get()).asWPILibState(), /*frc::Rotation2d(units::radian_t(Deg2Rad(90)))*/
        m_Trajectory.sample(m_Timer.Get()).holonomicRotation);

    speeds.vy = -speeds.vy;
    speeds.omega = -speeds.omega;

    r->GetDriveTrain().BaseDrive(
        speeds
    );
}

void DriveToPosCommand::End(bool interrupted){
    DebugOutF("Ending follow");
    m_Timer.Stop();
    Robot::GetRobot()->GetDriveTrain().m_DriveToPoseFlag = false;

}

//End command when close to intended pose
bool DriveToPosCommand::IsFinished(){
    return false;//m_Trajectory.getTotalTime() + 1.5_s < m_Timer.Get();
}