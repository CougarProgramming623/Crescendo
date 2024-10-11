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

        void SetVoltage(double driveVoltage, double steerAngle);
        void SetVelocity(double targetVelocity, double steerAngle);
        void BrakeMode(bool on);

        int dID;
        int sID;
    
        DriveController m_DriveController;
        SteerController m_SteerController;
};
