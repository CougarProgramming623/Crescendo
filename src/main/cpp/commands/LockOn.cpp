#include "commands/LockOn.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "./subsystems/Drivetrain.h"

LockOn::LockOn() {
    //AddRequirements(&Robot::GetRobot()->GetDriveTrain());
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
        m_AprilTagID = m_LimelightTable->GetNumber("tid", 0.0);
        Robot::GetRobot()->GetVision().setPriority(m_AprilTagID);   
        DebugOutF(std::to_string(m_LimelightTable->GetNumber("priorityid", 0.0)));
        //DebugOutF("robot has found april tag");
        r->GetVision().CalcPose();
        //m_GoalTheta = Rotation2d(r->GetVision().VisionRobotYaw(m_AprilTagID));
        m_GoalTheta = Rotation2d(units::angle::degree_t(r->GetAngle() + m_LimelightTable->GetNumber("tx", 0.0)));

        //print statements
        //DebugOutF("Robot Angle: " + std::to_string(r->GetAngle()));
        //DebugOutF("April Tag ID: " + std::to_string(m_AprilTagID));
        // DebugOutF("Target Robot Angle: " + std::to_string(m_GoalTheta.Degrees().value()));
        // DebugOutF("Shooter Angle: " + std::to_string(r->GetVision().ShooterAngle(m_AprilTagID).value()));
    //}
    
    //DebugOutF("Bot Pose: " + std::to_string(r->GetVision().GetFieldPose()));
    //DebugOutF(std::to_string(fmod(360 + 90 - r->GetNavX().GetAngle(), 360)));

    //DebugOutF("Estimated Robot Angle: " + std::to_string(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));

    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.02) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
            units::meters_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(1), 0.02) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
            //units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), frc::Pose2d(0_m, 0_m, m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega() * .45),
            units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), frc::Pose2d(0_m, 0_m, m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega() * .45),
            frc::Rotation2d(units::radian_t(Deg2Rad(r->GetAngle())))
    );

    // DebugOutF("OdoX: " + std::to_string(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
    // DebugOutF("OdoY: " + std::to_string(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
    // DebugOutF("OdoZ: " + std::to_string(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));

    // DebugOutF("Robot Angle: " + std::to_string(r->GetAngle()));
    m_AngleError = r->GetAngle() - m_GoalTheta.Degrees().value();
    // speeds.omega = -speeds.omega;
    
    r->GetDriveTrain().BaseDrive(speeds);
    }

    else{
        Robot* r = Robot::GetRobot();
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                units::meters_per_second_t(-Deadfix(r->GetJoyStick().GetRawAxis(1), 0.05) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
                units::meters_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.05) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
                units::radians_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(2), 0.05) * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 - r->GetNavX().GetAngle(), 360))))
        );
        r->GetDriveTrain().BaseDrive(speeds);
        //DriveWithJoystick();

    }
    //DebugOutF("Robot Angle after turning: " + std::to_string(r->GetNavX().GetAngle()));
}

bool LockOn::IsFinished() {
    return Robot::GetRobot()->m_AutoFlag && abs(m_AngleError) < 0.5;
}