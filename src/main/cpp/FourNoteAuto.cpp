#include <pathplanner/lib/commands/PathPlannerAuto.h>

using namespace pathplanner;

frc2::CommandPtr Robot::GetRobot()::getAutonomousCommand(){
    return PathPlannerAuto("Example Auto").ToPtr();
}