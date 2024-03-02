#include "SteerController.h"
#include "Constants.h"

//Constructor
SteerController::SteerController(int motorID, int EncoderPort, double AngleOffset):
    motor(motorID),
    encoder{EncoderPort},
    angleOffsetDegrees(AngleOffset)
{
    // DebugOutF(std::to_string((360-(fmod(((encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360-AngleOffset)), 360))) / STEER_ENCODER_POSITION_CONSTANT));
    // ratio.SensorToMechanismRatio = 150/7;
    // motor.GetConfigurator().Apply(ratio);
    motor.SetPosition(units::angle::turn_t((360-(fmod(((encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360-AngleOffset)), 360))) / STEER_ENCODER_POSITION_CONSTANT));
    //motor.SetInverted(true);
}

//Returns the reference angle which is just like not useful in radians
double SteerController::GetReferenceAngle() {return referenceAngleRadians;}

//Returns the angle of the module in radians
double SteerController::GetStateAngle(){ //gets the current angle of the motor
    double motorAngleRadians = motor.GetPosition().GetValueAsDouble() * STEER_ENCODER_POSITION_CONSTANT;
    motorAngleRadians = fmod(motorAngleRadians, 2.0 * M_PI);
    if(motorAngleRadians < 0.0){
        motorAngleRadians += 2.0 * M_PI;
    }
    return motorAngleRadians;
}

//Moves the module to the correct angle
void SteerController::SetReferenceAngle(double referenceAngleRadians) {
    double currentAngleRadians = motor.GetPosition().GetValueAsDouble() * STEER_ENCODER_POSITION_CONSTANT;
    // DebugOutF("raw current radians: " + std::to_string(currentAngleRadians));
    // DebugOutF("motor position in ticks" + std::to_string(motor.GetPosition().GetValueAsDouble()));

    // if(motor.GetVelocity().GetValueAsDouble() * STEER_ENCODER_VELOCITY_CONSTANT < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
    //     if(++resetIteration >= ENCODER_RESET_ITERATIONS) {
    //         resetIteration = 0;
    //         double absoluteAngle = Deg2Rad(360-(fmod(((encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360-angleOffsetDegrees)), 360))) / STEER_ENCODER_POSITION_CONSTANT;
    //         DebugOutF(std::to_string(absoluteAngle));
    //         motor.SetPosition(units::angle::turn_t(absoluteAngle / STEER_ENCODER_POSITION_CONSTANT));
    //         currentAngleRadians = absoluteAngle;
    //     }
    // } else{ 
    //     resetIteration = 0;
    // }

    double currentAngleRadiansMod = fmod(currentAngleRadians, (2.0 * M_PI));
    if(currentAngleRadiansMod < 0.0) {
        currentAngleRadiansMod += (2.0 * M_PI);
    }

    double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if(referenceAngleRadians - currentAngleRadiansMod > M_PI) {
        adjustedReferenceAngleRadians -= (2.0 * M_PI);
    } else if(referenceAngleRadians - currentAngleRadiansMod < -M_PI) {
        adjustedReferenceAngleRadians += (2.0 * M_PI);
    }

    // DebugOutF("current rotations: " + std::to_string(motor.GetPosition().GetValueAsDouble()));

    // DebugOutF("target rotations: " + std::to_string(adjustedReferenceAngleRadians / STEER_ENCODER_POSITION_CONSTANT));
    motor.SetControl(motorControlMode.WithPosition(units::angle::turn_t((adjustedReferenceAngleRadians / STEER_ENCODER_POSITION_CONSTANT))));
    // motor.SetPosition(units::angle::turn_t((adjustedReferenceAngleRadians / STEER_ENCODER_POSITION_CONSTANT)), 0.01_s);
    // motor.SetVoltage(6_V);

    //this.referenceAngleRadians = referenceAngleRadians;
}
    