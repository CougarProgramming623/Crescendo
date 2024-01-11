#include "DriveController.h"
#include "Constants.h"
using ctre::phoenix::motorcontrol::ControlMode;
using ctre::phoenix::motorcontrol::NeutralMode;

//Constructor
DriveController::DriveController(int ID)
:
    motor(ID)
{
    motor.SetNeutralMode(NeutralMode::Brake);
    //motor.SetInverted(SMTH)                           FIX idk why but some of them rotate clockwise and others counter clockwise
    motor.SetSensorPhase(true);                         //FIX also dont know why we do this one
}

//Set drive voltage
void  DriveController::SetReferenceVoltage(double voltage){
    motor.Set(ControlMode::PercentOutput, voltage / nominalVoltage);
}   

//Get module velocity in meters per second
double DriveController::GetStateVelocity(){
    return motor.GetSelectedSensorVelocity() * DRIVE_ENCODER_VELOCITY_CONSTANT;
}

//Set break mode
void DriveController::BreakMode(bool on){
    if(on)
        motor.SetNeutralMode(NeutralMode::Brake);
    else    
        motor.SetNeutralMode(NeutralMode::Coast);
}
