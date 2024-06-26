#include "commands/LockOn.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "./subsystems/Drivetrain.h"

LockOn::LockOn() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void LockOn::Initialize(){
    Robot::GetRobot()->GetDriveTrain().LockOnStatus = false;
}

//joystick deadzone -> if input is below deadband, set input to zero
double LockOn::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}

double LockOn::cubicMod(double in, double cm) {
    return cm * pow(in, 3) + (1 - cm) * in;
}

//if the limelight detects a target, robot theta and (LATER) shooter locks onto april tag
void LockOn::Execute() {    
    Robot* r = Robot::GetRobot();
    std::shared_ptr<nt::NetworkTable> limelight = r->GetVision().GetLimeLight();
    if(limelight->GetNumber("tv", 0.0) == 1) {
        m_AprilTagID = limelight->GetNumber("tid", 0.0);
        // r->GetVision().CalcPose();
        double LLCenterOffset = Rad2Deg(asin(LIMELIGHT_CENTER_DISPLACEMENT/r->GetVision().DistanceFromAprilTag(m_AprilTagID)));
        m_GoalTheta = Rotation2d(units::degree_t(r->GetAngle() + limelight->GetNumber("tx", 0.0) + LLCenterOffset));

        //print statements
        // DebugOutF("April Tag ID: " + std::to_string(m_AprilTagID));
        DebugOutF("Target Robot Angle: " + std::to_string(m_GoalTheta.Degrees().value()));
        DebugOutF("Robot Angle: " + std::to_string(r->GetAngle()));

        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(-cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(1), 0.02), 0.3) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
            units::meters_per_second_t(cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.02), 0.3) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
            // units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), frc::Pose2d(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Translation(), m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega()),
            units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(frc::Pose2d(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Translation(), Rotation2d(units::degree_t(r->GetAngle()))), frc::Pose2d(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Translation(), m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega()),
            frc::Rotation2d(units::radian_t(Deg2Rad(-r->GetAngle())))
        );

        m_AngleError = r->GetAngle() - m_GoalTheta.Degrees().value();
        //DebugOutF("Angle Error: " + std::to_string(m_AngleError));

        // speeds.omega = -speeds.omega;
        DebugOutF("omega: " + std::to_string(speeds.omega()));
        r->GetDriveTrain().BaseDrive(speeds);

        if(abs(m_AngleError) < 1) {
            Robot::GetRobot()->GetDriveTrain().LockOnStatus = true;
        } else {
            Robot::GetRobot()->GetDriveTrain().LockOnStatus = false;
        }

    } else {
        Robot* r = Robot::GetRobot();
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                units::meters_per_second_t(-cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(1), 0.05), 0.3) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
                units::meters_per_second_t(cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.05), 0.3) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
                units::radians_per_second_t(cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(2), 0.05), 0.75) * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                frc::Rotation2d(units::radian_t(Deg2Rad(-r->GetAngle())))
        );
        r->GetDriveTrain().BaseDrive(speeds);
    }
}

void LockOn::End(bool interrupted) {
    Robot::GetRobot()->GetDriveTrain().LockOnStatus = false;
    //DebugOutF(std::to_string(Robot::GetRobot()->GetDriveTrain().LockOnStatus));
}

bool LockOn::IsFinished() {
    return Robot::GetRobot()->m_AutoFlag && abs(m_AngleError) < 0.5;
}