#include "./commands/AutoTest.h"
#include "Robot.h"


void AutoTest::Initialize(){};

void AutoTest::Execute() {
    Robot::GetRobot()->GetDriveTrain().m_FrontRightModule.SetVoltage(9.0, 0.0);
    DebugOutF("testestest");
};

void AutoTest::End(bool isFinished){};

