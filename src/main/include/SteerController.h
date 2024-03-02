#pragma once

#include "Util.h"

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/AnalogInput.h>


using namespace ctre::phoenix6;
//using ctre::phoenix::motorcontrol::ControlMode;

class SteerController {
    public:
        SteerController(int motorID, int EncoderPort, double AngleOffset);

        double GetReferenceAngle();
        double GetStateAngle();

        void SetReferenceAngle(double referenceAngleRadians);
        
        hardware::TalonFX motor;
        controls::PositionDutyCycle motorControlMode{units::angle::turn_t(0)};
        
        frc::AnalogInput encoder;
        
        double angleOffsetVoltage;
        double referenceAngleRadians;
        double resetIteration;

};