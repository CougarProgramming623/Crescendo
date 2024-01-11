#include "SwerveModule.h"
#include "Util.h"
#include "Constants.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/geometry/Rotation2d.h"

//Constructor
SwerveModule::SwerveModule(int driveID, int steerID, int encoderPort, double angleOffset):
    m_DriveController(driveID), 
    m_SteerController(steerID, encoderPort, angleOffset)
{}

//Get the velocity of the module in meters per second
double SwerveModule::GetDriveVelocity(){
    return m_DriveController.GetStateVelocity();
}

//Get the angle of the module in radians
double SwerveModule::GetSteerAngle(){
    return m_SteerController.GetStateAngle();
}

//Set break mode of the drive motor
void SwerveModule::BreakMode(bool on){
    m_DriveController.BreakMode(on);
}

//Get the pose of the module
frc::SwerveModulePosition SwerveModule::GetPosition(){
    return {units::meter_t(m_DriveController.motor.GetSelectedSensorPosition() * DRIVE_ENCODER_POSITION_CONSTANT), frc::Rotation2d(units::radian_t(-GetSteerAngle()))};
}

//Set the module to drive at a voltage at an angle in radians
void SwerveModule::Set(double driveVoltage, double steerAngle){
    steerAngle  = fmod(steerAngle, 2.0 * M_PI);
    if(steerAngle < 0.0){
        steerAngle += (2.0 * M_PI);
    }

    double difference = steerAngle - GetSteerAngle();

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

    m_SteerController.SetReferenceAngle(steerAngle);
    m_DriveController.SetReferenceVoltage(driveVoltage);
}