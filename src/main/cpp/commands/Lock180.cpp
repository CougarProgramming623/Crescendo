#include "commands/Lock180.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "./subsystems/Drivetrain.h"

Lock180::Lock180() {
    // m_GoalTheta = frc::Rotation2d(units::degree_t(angle));
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

double Lock180::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}

double Lock180::cubicMod(double in, double cm) {
    return cm * pow(in, 3) + (1 - cm) * in;
}

void Lock180::Initialize() {
    m_GoalTheta = frc::Rotation2d(units::degree_t(Robot::GetRobot()->GetAngle()));
}

void Lock180::Execute() {
    Robot* r = Robot::GetRobot();
    double angle = r->GetAngle();
    // DebugOutF("angle: " + std::to_string(angle));
    // if(angle < 90 || angle > 270) {
    //     m_GoalTheta = frc::Rotation2d(units::degree_t(0));
    // } else {
    //     m_GoalTheta = frc::Rotation2d(units::degree_t(180));
    // }

    //prints
    DebugOutF("goal theta: " + std::to_string(m_GoalTheta.Degrees().value()));
    DebugOutF("current angle: " + std::to_string(angle));

    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t(-cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(1), 0.05), 0.5) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
        units::meters_per_second_t(cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.05), 0.5) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
        // units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), frc::Pose2d(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Translation(), m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega()),
        units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(frc::Pose2d(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Translation(), Rotation2d(units::degree_t(r->GetAngle()))), frc::Pose2d(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Translation(), m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega()),
        frc::Rotation2d(units::radian_t(Deg2Rad(-r->GetAngle())))
    );

    speeds.omega = -speeds.omega;
    DebugOutF("omega: " + std::to_string(speeds.omega()));
    r->GetDriveTrain().BaseDrive(speeds);
}

void Lock180::End(bool interrupted) {
    Robot::GetRobot()->GetDriveTrain().BaseDrive(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
}