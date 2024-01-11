#pragma once

#include "Util.h"

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/AnalogInput.h>


using ctre::phoenix::motorcontrol::can::TalonFX;
using ctre::phoenix::motorcontrol::ControlMode;

class SteerController {
    public:
        SteerController(int motorID, int EncoderPort, double AngleOffset);

        double GetReferenceAngle();
        double GetStateAngle();

        void SetReferenceAngle(double referenceAngleRadians);

        TalonFX motor;
        ControlMode motorControlMode;
        
        frc::AnalogInput encoder;

        double angleOffsetDegrees;
        double referenceAngleRadians;
        double resetIteration;

};