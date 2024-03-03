#include "SteerController.h"
#include "Constants.h"
#include <frc/RobotController.h>

using namespace ctre::phoenix6::controls;

//Constructor
SteerController::SteerController(int motorID, int EncoderPort, double AngleOffset):
    motor(motorID),
    encoder{EncoderPort},
    angleOffsetVoltage(AngleOffset)
{
    //motor.SetControl(motorControlMode.WithPosition(units::angle::turn_t((360-(fmod(((encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360-AngleOffset)), 360))) / STEER_ENCODER_POSITION_CONSTANT)));
    DebugOutF("initial position: " + std::to_string((encoder.GetVoltage() / frc::RobotController::GetVoltage5V()) - (angleOffsetVoltage / MAX_VOLTAGE_WHEN_OFFSET)));
    motor.SetPosition(units::angle::turn_t((angleOffsetVoltage / MAX_VOLTAGE_WHEN_OFFSET) - (encoder.GetVoltage() / frc::RobotController::GetVoltage5V())));
    motorControlMode.Position = units::turn_t((encoder.GetVoltage() / frc::RobotController::GetVoltage5V()) - (angleOffsetVoltage / MAX_VOLTAGE_WHEN_OFFSET));
}

//Returns the reference angle which is just like not useful in radians
double SteerController::GetReferenceAngle() {return referenceAngleRadians;}

//Returns the angle of the module in radians
double SteerController::GetStateAngle(){ //gets the current angle of the motor
    // DebugOutF("current motor rotations: " + std::to_string(motor.GetPosition().GetValueAsDouble()));
    double motorAngleRadians = motor.GetPosition().GetValueAsDouble() * STEER_ENCODER_POSITION_CONSTANT;
    motorAngleRadians = fmod(motorAngleRadians, 2.0 * M_PI);
    if(motorAngleRadians < 0.0){
        motorAngleRadians += 2.0 * M_PI;
    }
    return motorAngleRadians;
}

//Moves the module to the correct angle
void SteerController::SetReferenceAngle(double referenceAngleRadians){
    // DebugOutF("current angle before point 3 in ROTATIONS: " + std::to_string(motor.GetPosition().GetValueAsDouble()));
    double currentAngleRadians = motor.GetPosition().GetValueAsDouble() * STEER_ENCODER_POSITION_CONSTANT;
    // DebugOutF("current angle before point 3 in radians: " + std::to_string(currentAngleRadians));
    // DebugOutF("motor position in ticks" + std::to_string(motor.GetPosition().GetValueAsDouble()));

    // if(motor.GetVelocity().GetValueAsDouble() * STEER_ENCODER_VELOCITY_CONSTANT < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
    //     if(++resetIteration >= ENCODER_RESET_ITERATIONS) {
    //         resetIteration = 0;
    //         double absoluteAngle = Deg2Rad(360-(fmod(((encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360-angleOffsetDegrees)), 360))) / STEER_ENCODER_POSITION_CONSTANT;
    //         // DebugOutF(std::to_string(absoluteAngle));
    //         // motor.SetSelectedSensorPosition(absoluteAngle / STEER_ENCODER_POSITION_CONSTANT);
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
    // DebugOutF("current angle before point 3 in radians AFTER MOD: " + std::to_string(currentAngleRadiansMod - currentAngleRadiansMod));

    double adjustedReferenceAngleRadians = referenceAngleRadians;// + currentAngleRadians - currentAngleRadiansMod;
    if(referenceAngleRadians - currentAngleRadiansMod > M_PI) {
        adjustedReferenceAngleRadians -= (2.0 * M_PI);
    } else if(referenceAngleRadians - currentAngleRadiansMod < -M_PI) {
        adjustedReferenceAngleRadians += (2.0 * M_PI);
    }

    motor.SetControl(motorControlMode.WithPosition(units::angle::turn_t(adjustedReferenceAngleRadians / STEER_ENCODER_POSITION_CONSTANT)));

    //this.referenceAngleRadians = referenceAngleRadians;
}
    