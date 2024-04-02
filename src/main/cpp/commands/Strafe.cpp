#include "commands/Strafe.h"
#include "Robot.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/DriverStation.h"

Strafe::Strafe(int direction) : 
    m_Timer()
{
    AddRequirements(&Robot::GetRobot()->GetDriveTrain());
    if(direction == 0) {
        m_Direction = 1;
    } else if(direction == 1) {
        m_Direction = -1;
    }
}

//If input is below deadband, set to zero
double Strafe::Deadfix(double in, double deadband) {
    if(abs(in) < deadband) {
        return 0;
    }
    return in;
}

double Strafe::cubicMod(double in, double cm) {
    return cm * pow(in, 3) + (1 - cm) * in;
}

void Strafe::Initialize() {
    m_Timer.Reset();
    m_Timer.Start();
}

void Strafe::Execute() {
    // if(frc::DriverStation::GetMatchTime().value() > ) {
        Robot* r = Robot::GetRobot();
    
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
            units::meters_per_second_t(-cubicMod(Deadfix(r->GetJoyStick().GetRawAxis(1), 0.05), 0.8) * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND),
            units::meters_per_second_t(r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND * 0.15 * m_Direction),
            units::radians_per_second_t(0),
            frc::Rotation2d(units::radian_t(Deg2Rad(fmod(360 - r->GetNavX().GetAngle(), 360))))
        );

        r->GetDriveTrain().BaseDrive(speeds);
    // }
    
}

void Strafe::End(bool interrupted) {
    Robot::GetRobot()->GetDriveTrain().BaseDrive(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));
}

bool Strafe::IsFinished() {
    return false;
    // return m_Timer.Get() >= 0.2_s;
};