#pragma once

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
//#include <ctre/phoenix/motorcontrol/StatusFrame>
//#include <ctre/phoenix/motorcontrol/TalonFXControlMode.h>
//#include <ctre/phoenix/motorcontrol/TalonFXInvertType.h>
#include <ctre/phoenix6/TalonFX.hpp>
//#include <ctre/phoenix/motorcontrol/can/TalonFXConfiguration.h>

using namespace ctre::phoenix6;

class DriveController {
    public:
        DriveController(int ID);

        void SetReferenceVoltage(double voltage);
        //void GetReferenceVoltage(double voltage);

        double GetStateVelocity();

        inline hardware::TalonFX& GetMotor() { return motor; }

        void BreakMode(bool on);
        hardware::TalonFX motor;

    private:
        double nominalVoltage = 12;  //FIX it is double.NaN in the java and i still dont know what that means
        double currentLimit;    //FIX it is double.NaN in the java and i still dont know what that means


};  