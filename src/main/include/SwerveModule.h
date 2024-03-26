#pragma once

#include "DriveController.h"
#include "SteerController.h"
#include "frc/kinematics/SwerveModulePosition.h"


class SwerveModule {
    public:
        SwerveModule(int driveID, int steerID, int encoderPort, double angleOffset);
        SwerveModule();

        double GetDriveVelocity();

        double GetSteerAngle();

        double GetSteerSensorVoltage();

        frc::SwerveModulePosition GetPosition();

        void Set(double driveVoltage, double steerAngle);
        void BrakeMode(bool on);
    
        DriveController m_DriveController;
        SteerController m_SteerController;
};
