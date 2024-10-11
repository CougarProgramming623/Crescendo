// #include "commands/TrajectoryCommand.h"
// #include "Robot.h"
// #include "frc/kinematics/ChassisSpeeds.h"
// #include "Util.h"

// using namespace pathplanner;


// //Constructs a Trajectory Command based on a PathPlannerTrajectory
// TrajectoryCommand::TrajectoryCommand(PathPlannerTrajectory trajectory) {
//     // AddRequirements(&Robot::GetRobot()->GetDriveTrain());
//     // m_Trajectory = trajectory;
// }

// //Start timer
// void TrajectoryCommand::Initialize(){
//     Robot::GetRobot()->GetDriveTrain().GetTimer().Reset();
//     Robot::GetRobot()->GetDriveTrain().GetTimer().Start();
// }

// /*
// Calculates required chassis speed to match trajactory
// Passes ChassisSpeed object to BaseDrive() function
// */
// void TrajectoryCommand::Execute() {
//     DebugOutF("Execute");
//     Robot* r = Robot::GetRobot();
//     frc::Timer timer = r->GetDriveTrain().GetTimer();
//     frc::ChassisSpeeds speeds = r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetPose(), m_Trajectory.sample(timer.Get()).getTargetHolonomicPose(), m_Trajectory.sample(timer.Get()).velocity, m_Trajectory.sample(timer.Get()).getTargetHolonomicPose().Rotation().Degrees());
//     DebugOutF("end heading: " + std::to_string(m_Trajectory.getStates().at(m_Trajectory.getStates().size() - 1).heading.Degrees().value()));
//     DebugOutF("current angle: " + std::to_string(r->GetAngle()));
//     DebugOutF("target heading: " + std::to_string(m_Trajectory.sample(timer.Get()).getTargetHolonomicPose().Rotation().Degrees().value()));
//     DebugOutF("timer: " + std::to_string(timer.Get().value()));

//     // speeds.vy = -speeds.vy;
//     speeds.omega = -speeds.omega;
//     DebugOutF("current omega: " + std::to_string(speeds.omega.value()));

//     r->GetDriveTrain().BaseDrive(speeds);
// }

// void TrajectoryCommand::End(bool interrupted){
//     //DebugOutF("Ending follow");
//     Robot::GetRobot()->GetDriveTrain().GetTimer().Stop();
//     Robot::GetRobot()->GetDriveTrain().SetChassisSpeeds(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
// }

// //End command when close to intended pose
// bool TrajectoryCommand::IsFinished(){
//     return m_Trajectory.getTotalTime() + 0.5_s < Robot::GetRobot()->GetDriveTrain().GetTimer().Get();
// }