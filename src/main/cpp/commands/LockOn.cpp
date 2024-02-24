#include "commands/LockOn.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "./subsystems/Drivetrain.h"

LockOn::LockOn() {
    DebugOutF("inside LOCKON!!!");
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void LockOn::Initialize(){}

//joystick deadzone -> if input is below deadband, set input to zero
double LockOn::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}

//if the limelight detects a target, robot theta and (LATER) shooter locks onto april tag
void LockOn::Execute() {    
    Robot* r = Robot::GetRobot();
    if(m_LimelightTable->GetNumber("tv", 0.0) == 1) {
        DebugOutF("robot has found april tag");
        m_AprilTagID = m_LimelightTable->GetNumber("tid", 0.0);
        m_GoalTheta = r->GetVision().VisionRobotYaw(r->GetVision().GetFieldPose(), m_LimelightTable->GetNumberArray("tid",std::vector<double>(6))[0]);
    }
    
    //print statements
    DebugOutF("Robot Angle: " + std::to_string(fmod(360 + 90 - r->GetNavX().GetAngle(), 360)));
    DebugOutF("April Tag ID: " + std::to_string(m_AprilTagID));
    DebugOutF("Target Robot Angle: " + std::to_string(m_GoalTheta.Degrees().value()));
    DebugOutF("Shooter Angle: " + std::to_string(r->GetVision().ShooterAngle(r->GetVision().GetFieldPose(), m_LimelightTable->GetNumberArray("tid",std::vector<double>(6))[0]).value()));
    //DebugOutF("Bot Pose: " + std::to_string(r->GetVision().GetFieldPose()));
    //DebugOutF("Act: " + std::to_string(r->GetAngle()));
    //DebugOutF(std::to_string(fmod(360 + 90 - r->GetNavX().GetAngle(), 360)));

    // frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //         units::meters_per_second_t(-Deadfix(r->GetJoyStick().GetRawAxis(1), 0.02) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
    //         units::meters_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.02) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
    //         units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), frc::Pose2d(0_m, 0_m, m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega() * .45),
    //         frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 - r->GetNavX().GetAngle(), 360))))
    // );

    // speeds.omega = -speeds.omega;
    
    // r->GetDriveTrain().BaseDrive(speeds);
}