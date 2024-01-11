#include "commands/AutoLock.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "./subsystems/Drivetrain.h"

AutoLock::AutoLock() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void AutoLock::Initialize(){}

//If input is below deadband, set to zero
double AutoLock::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}

//Take joystick input, convert to ChassisSpeeds object, and pass to BaseDrive() function
void AutoLock::Execute() {    
    Robot* r = Robot::GetRobot();
    ((r->GetAngle() > 90) && (r->GetAngle() < 270)) ? m_GoalTheta = frc::Rotation2d(units::degree_t(180)) : m_GoalTheta = frc::Rotation2d(units::radian_t(0));
    // DebugOutF(std::to_string(m_GoalTheta.Degrees().value()));
    // DebugOutF("Act: " + std::to_string(r->GetAngle()));
    //DebugOutF(std::to_string(fmod(360 + 90 - r->GetNavX().GetAngle(), 360)));
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(-Deadfix(r->GetJoyStick().GetRawAxis(1), 0.02) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
            units::meters_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.02) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.7),
            units::radians_per_second_t(r->GetDriveTrain().GetHolonomicController().Calculate(r->GetDriveTrain().GetOdometry()->GetEstimatedPosition(), frc::Pose2d(0_m, 0_m, m_GoalTheta), 0_m / 1_s, m_GoalTheta).omega() * .45),
            frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 - r->GetNavX().GetAngle(), 360))))
    );

    speeds.omega = -speeds.omega;
    
    r->GetDriveTrain().BaseDrive(speeds);
}