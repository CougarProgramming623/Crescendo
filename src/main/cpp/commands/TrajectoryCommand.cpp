#include "commands/TrajectoryCommand.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "Util.h"

using namespace pathplanner;


//Constructs a Trajectory Command based on a PathPlannerTrajectory
TrajectoryCommand::TrajectoryCommand(PathPlannerTrajectory trajectory) {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
    m_Trajectory = trajectory;
}

//Start timer
void TrajectoryCommand::Initialize(){
    Robot::GetRobot()->GetDriveTrain().GetTimer().Reset();
    Robot::GetRobot()->GetDriveTrain().GetTimer().Start();
}

/*
Calculates required chassis speed to match trajactory
Passes ChassisSpeed object to BaseDrive() function
*/
void TrajectoryCommand::Execute() {
    DebugOutF("Execute");
    Robot* r = Robot::GetRobot();
    frc::Timer timer = r->GetDriveTrain().GetTimer();
    frc::ChassisSpeeds speeds = r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), m_Trajectory.sample(timer.Get()).getTargetHolonomicPose(), m_Trajectory.sample(timer.Get()).velocity, m_Trajectory.getEndState().heading);

    // speeds.vy = -speeds.vy;
    // speeds.omega = -speeds.omega;

    r->GetDriveTrain().BaseDrive(speeds);
}

void TrajectoryCommand::End(bool interrupted){
    //DebugOutF("Ending follow");
    Robot::GetRobot()->GetDriveTrain().GetTimer().Stop();
    Robot::GetRobot()->GetDriveTrain().SetChassisSpeeds(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
}

//End command when close to intended pose
bool TrajectoryCommand::IsFinished(){
    return m_Trajectory.getTotalTime() + 0.25_s < Robot::GetRobot()->GetDriveTrain().GetTimer().Get();
}