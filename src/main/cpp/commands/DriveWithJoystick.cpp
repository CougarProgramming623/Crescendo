#include "commands/DriveWithJoystick.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"

DriveWithJoystick::DriveWithJoystick() {
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
    rotateMod = 0.85;
    translationMod = 0.5;
    Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/rotateMod").SetDouble(0.0);
    Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/translationeMod").SetDouble(0.0);
}

void DriveWithJoystick::Initialize(){}

//If input is below deadband, set to zero
double DriveWithJoystick::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}

double DriveWithJoystick::cubicMod(double in, double cm) {
    return cm * pow(in, 3) + (1 - cm) * in;
}

//Take joystick input, convert to ChassisSpeeds object, and pass to BaseDrive() function
void DriveWithJoystick::Execute() {
    Robot* r = Robot::GetRobot();
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t(-cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(1), 0.05), translationMod) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
            units::meters_per_second_t(cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(0), 0.05), translationMod) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
            units::radians_per_second_t(cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(2), 0.05), rotateMod) * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
            frc::Rotation2d(units::radian_t(Deg2Rad(-r->GetAngle())))
    );

    //DebugOutF("omega: " + std::to_string(speeds.omega()));
    
    r->GetDriveTrain().BaseDrive(speeds);
}