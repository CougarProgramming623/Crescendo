#include "commands/DriveWithJoystick.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"

DriveWithJoystick::DriveWithJoystick() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
}

void DriveWithJoystick::Initialize(){}

//If input is below deadband, set to zero
double DriveWithJoystick::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}

//Take joystick input, convert to ChassisSpeeds object, and pass to BaseDrive() function
void DriveWithJoystick::Execute() {
    Robot* r = Robot::GetRobot();
    //DebugOutF(std::to_string(fmod(360 + 90 - r->GetNavX().GetAngle(), 360)));
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(-Deadfix(r->GetJoyStick().GetRawAxis(1), 0.03) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
            units::meters_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.03) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
            units::radians_per_second_t(Deadfix(r->GetJoyStick().GetRawAxis(2), 0.02) * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
            frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 - r->GetNavX().GetAngle(), 360))))
    );

    // if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    //     speeds.vx = -speeds.vx;
    //     speeds.vy = -speeds.vy;
    // }
    //Auto Pose Fix
    r->GetDriveTrain().BaseDrive(speeds);
}