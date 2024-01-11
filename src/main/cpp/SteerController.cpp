#include "SteerController.h"
#include "Constants.h"

//Constructor
SteerController::SteerController(int motorID, int EncoderPort, double AngleOffset):
    motor(motorID),
    encoder{EncoderPort},
    motorControlMode{ctre::phoenix::motorcontrol::ControlMode::Position},
    angleOffsetDegrees(AngleOffset)
{
    motor.SetSelectedSensorPosition(Deg2Rad(360-(fmod(((encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360-AngleOffset)), 360))) / STEER_ENCODER_POSITION_CONSTANT);
}

//Returns the reference angle which is just like not useful in radians
double SteerController::GetReferenceAngle() {return referenceAngleRadians;}

//Returns the angle of the module in radians
double SteerController::GetStateAngle(){ //gets the current angle of the motor
    double motorAngleRadians = motor.GetSelectedSensorPosition() * STEER_ENCODER_POSITION_CONSTANT;
    motorAngleRadians = fmod(motorAngleRadians, 2.0 * M_PI);
    if(motorAngleRadians < 0.0){
        motorAngleRadians += 2.0 * M_PI;
    }
    return motorAngleRadians;
}

//Moves the module to the correct angle
void SteerController::SetReferenceAngle(double referenceAngleRadians){
    double currentAngleRadians = motor.GetSelectedSensorPosition() * STEER_ENCODER_POSITION_CONSTANT;

    // if(motor.GetSelectedSensorVelocity() * STEER_ENCODER_VELOCITY_CONSTANT < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
    //     if(++resetIteration >= ENCODER_RESET_ITERATIONS) {
    //         resetIteration = 0;
    //         double absoluteAngle = Deg2Rad(360-(fmod(((encoder.GetVoltage() * ENCODER_VOLTAGE_TO_DEGREE) + (360-angleOffsetDegrees)), 360))) / STEER_ENCODER_POSITION_CONSTANT;
    //         motor.SetSelectedSensorPosition(absoluteAngle / STEER_ENCODER_POSITION_CONSTANT);
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

    motor.Set(motorControlMode, adjustedReferenceAngleRadians / STEER_ENCODER_POSITION_CONSTANT);

    //this.referenceAngleRadians = referenceAngleRadians;
}
    