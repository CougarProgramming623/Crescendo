#include "LED.h"
#include "Util.h"
#include "Constants.h"
#include "Robot.h"
#include "frc/RobotBase.h"
#include "frc/Timer.h"
#include "frc/DriverStation.h"

LED::LED() {}

void LED::Init(){
    DebugOutF("LED Init");
    m_AddressableLED.SetLength(numLEDs);
    m_AddressableLED.Start();
    m_IterationTracker = 0;
    m_IsTele = false;
    SponsorBoardAllianceColor();
}

void LED::SetData() { m_AddressableLED.SetData(m_LEDBuffer); }

void LED::SponsorBoardAllianceColor() {
    // if(Robot::GetRobot()->GetCOB().GetTable().GetEntry("/FMSInfo/IsRedAlliance").GetBoolean(false)) {
    //     m_AllianceColor = frc::Color::kRed;
    // } else {
    //     m_AllianceColor = frc::Color::kBlue;
    // }
    // if (frc::DriverStation::GetAlliance() != frc::DriverStation::Alliance::kRed &&
    //     frc::DriverStation::GetAlliance() != frc::DriverStation::Alliance::kBlue) {
    //     m_AllianceColor = frc::Color::kWhite;
    // }
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
        m_AllianceColor = frc::Color::kRed;
    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        m_AllianceColor = frc::Color::kBlue;
    } else {
        m_AllianceColor = frc::Color::kWhite;
    }
    SponsorBoardSolid(m_AllianceColor);
}

void LED::SponsorBoardSolid(frc::Color color){
    for(int i = 0; i < numLEDs; i++) {
            m_LEDBuffer[i].SetLED(color);
    }
}

// void LED::UnderBotSensor() {
//     if(Robot::GetRobot()->m_UnderBotLaser.Get() == 0) {
//         for(int i = 0; i < numLEDs; i++) {
//             m_LEDBuffer[i].SetLED(frc::Color::kPurple);
//         }
//     } else {SponsorBoardAllianceColor();}
// }

void LED::LaserSensors() {
    if(Robot::GetRobot()->m_DustpanLaser.Get() == 0) {
        for(int i = 0; i < numLEDs; i++) {
            m_LEDBuffer[i].SetLED(frc::Color::kGreen);
        }
    } else if(Robot::GetRobot()->m_UnderBotLaser.Get() == 0) {
        for(int i = 0; i < numLEDs; i++) {
            m_LEDBuffer[i].SetLED(frc::Color::kPurple);
        }
    } else {SponsorBoardAllianceColor();}
}

// void LED::VisionCanSee() {
//     if(Robot::GetRobot()->GetVision().GetLimeLight()->GetNumber("tv", 0.0) == 1) {
//         if()
//     }
// }