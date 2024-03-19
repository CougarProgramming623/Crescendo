#include "DriveController.h"
#include "Constants.h"
#include "Robot.h"
// using ctre::phoenix::motorcontrol::ControlMode;
// using ctre::phoenix::motorcontrol::NeutralMode;
using namespace ctre::phoenix6;

//Constructor
DriveController::DriveController(int ID) :
    motor(ID)
{
    motor.SetNeutralMode(signals::NeutralModeValue::Brake);
}

//Set drive voltage
void DriveController::SetReferenceVoltage(double voltage){
    // if(invert) {
    //     motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(voltage / nominalVoltage * -1));
    // } else {
    motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(voltage / nominalVoltage));
    // }
    
}   

//Get module velocity in meters per second
double DriveController::GetStateVelocity(){
    return motor.GetVelocity().GetValueAsDouble() * DRIVE_ENCODER_VELOCITY_CONSTANT;
}

//set the drive motors to brake mode
void DriveController::BrakeMode(bool on){
    if(on)
        motor.SetNeutralMode(signals::NeutralModeValue::Brake);
    else    
        motor.SetNeutralMode(signals::NeutralModeValue::Coast);
}
