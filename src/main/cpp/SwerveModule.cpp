#include "SwerveModule.h"
#include "Util.h"
#include "Constants.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/geometry/Rotation2d.h"

//Constructor
SwerveModule::SwerveModule(int driveID, int steerID, int encoderPort, double angleOffset):
    m_DriveController(driveID), 
    m_SteerController(steerID, encoderPort, angleOffset)
{
    dID = driveID;
    sID = steerID;
}

//Get the velocity of the module in meters per second
double SwerveModule::GetDriveVelocity(){
    return m_DriveController.GetStateVelocity();
}

//Get the angle of the module in radians
double SwerveModule::GetSteerAngle(){
    return m_SteerController.GetStateAngle();
}

double SwerveModule::GetSteerSensorVoltage(){
    return m_SteerController.encoder.GetAverageVoltage();
}

//Set brake mode of the drive motor
void SwerveModule::BrakeMode(bool on){
    m_DriveController.BrakeMode(on);
}

//Get the pose of the module
frc::SwerveModulePosition SwerveModule::GetPosition() {
    // return {units::meter_t(m_DriveController.motor.GetPosition().GetValueAsDouble() * DRIVE_ENCODER_POSITION_CONSTANT), frc::Rotation2d(units::radian_t(-GetSteerAngle()))};
    units::meter_t distance = units::meter_t(m_DriveController.motor.GetPosition().GetValueAsDouble() * DRIVE_ENCODER_POSITION_CONSTANT);
    units::radian_t rot = units::radian_t(GetSteerAngle());
    if(dID == FRONT_LEFT_MODULE_DRIVE_MOTOR || dID == BACK_RIGHT_MODULE_DRIVE_MOTOR) {
        distance *= -1;
    }
    else if(sID == FRONT_RIGHT_MODULE_STEER_MOTOR || sID == BACK_LEFT_MODULE_STEER_MOTOR) {
        rot *= -1;
    }
    return {distance, frc::Rotation2d(rot)};
}

//Set the module to drive at a voltage at an angle in radians
void SwerveModule::Set(double driveVoltage, double steerAngle) {
    steerAngle  = fmod(steerAngle, 2.0 * M_PI);
    if(steerAngle < 0.0){
        steerAngle += (2.0 * M_PI);
    }

    // DebugOutF("steer angle of the robot in radians: " + std::to_string(GetSteerAngle()));
    double difference = steerAngle - GetSteerAngle();
    // DebugOutF("steer difference between desired and current: " + std::to_string(difference));

    if(difference >= M_PI) {
        steerAngle -= (2.0 * M_PI);
    }
    else if(difference < -M_PI) {
        steerAngle += (2.0 * M_PI);
    }

    difference = steerAngle - GetSteerAngle(); //recalc difference

    if(difference > M_PI / 2.0 || difference < (-M_PI) / 2.0) {
        steerAngle += M_PI;
        driveVoltage *= -1.0;
    }

    steerAngle = fmod(steerAngle, (2.0 * M_PI));
    if(steerAngle < 0.0) {
        steerAngle += (2.0 * M_PI);
    }

    // DebugOutF("steer angle at point 2 in radians: " + std::to_string(steerAngle));

    m_SteerController.SetReferenceAngle(steerAngle);
    m_DriveController.SetReferenceVoltage(driveVoltage);
}