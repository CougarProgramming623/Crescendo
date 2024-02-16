#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>
#include <Robot.h>

using namespace pathplanner;

Robot::GetRobot() : Robot::GetRobot()->GetDriveTrain(), Robot::GetRobot() {
    // Register Named Commands. You must pass either a CommandPtr rvalue or a shared_ptr to the command, not the command directly.
    NamedCommands::registerCommand("autoLock", std::move(AutoLock().ToPtr())); 
    NamedCommands::registerCommand("driveToPosCommand", std::move(DriveToPosCommand().ToPtr())); 
    NamedCommands::registerCommand("driveWithJoystick", std::move(DriveWithJoystick().ToPtr()));
    NamedCommands::registerCommand("dualMotorControl", std::move(DualMotorControl().ToPtr()));
    NamedCommands::registerCommand("dynamicIntake", std::move(DynamicIntake().ToPtr())); 
    NamedCommands::registerCommand("pivotToPos", std::move(PivotToPos().ToPtr())); 
    NamedCommands::registerCommand("pivotToPosAuto", std::move(PivotToPosAuto().ToPtr()));
    NamedCommands::registerCommand("trajectoryCommand", std::move(TrajectoryCommand().ToPtr()));
    NamedCommands::registerCommand("wristToPos", std::move(WristToPos().ToPtr())); 
    NamedCommands::registerCommand("wristToPosAuto", std::move(WristToPosAuto().ToPtr())); 

    // Do all other initialization
    configureButtonBindings();

    // ...
}