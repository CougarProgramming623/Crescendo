#include <pathplanner/lib/commands/FourNoteAuto.h>

using namespace pathplanner;

frc2::CommandPtr RobotContainer::getAutonomousCommand() {
    return PathPlannerAuto("Example Auto").ToPtr();
}