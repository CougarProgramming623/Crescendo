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
    m_AddressableLED.SetLength(numLEDsTotal);
    m_AddressableLED.Start();
    m_AddressableLED.SetData(m_LEDBuffer);
    m_AddressableLED.SetBitTiming(260_ns, 960_ns, 760_ns, 580_ns);
    m_AddressableLED.SetSyncTime(17_ms);
    m_IsTele = false;
    //LockOnStatus = Robot::GetRobot()->GetDriveTrain().LockOnStatus;
    // SponsorBoardAllianceColor();
}

void LED::SetData() {
    m_AddressableLED.SetData(m_LEDBuffer);
}

void LED::SponsorBoardAllianceColor() {
    // DebugOutF("first:" + std::to_string(Robot::GetRobot()->GetDriveTrain().LockOnStatus));
    //DebugOutF("second" + std::to_string(LockOnStatus));
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
        m_AllianceColor = red;
    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        m_AllianceColor = blue;
    } else {
        m_AllianceColor = white;
    }
    SponsorBoardSolid(m_AllianceColor);
}

void LED::SponsorBoardSolid(frc::Color color){
    for(int i = 0; i < m_LEDBuffer.size(); i++) {
        m_LEDBuffer[i].SetLED(color);
    }
    if(Robot::GetRobot()->GetVision().GetLimeLight()->GetNumber("tv", 0) == 1) {
        for (int i = 0; i < numLEDsLeftR1; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
        for(int i = numLEDsLeftEnd; i < numLEDsFrontR1; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
        for(int i = numLEDsFrontEnd; i < numLEDsRightR1; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
    }
}

void LED::LaserSensors() {
    // DebugOutF(std::to_string(LockOnStatus));
    // if(Robot::GetRobot()->m_DustpanLaser.Get() == 0) {
    //     for(int i = 0; i < numLEDs; i++) {
    //        m_LEDMiddle[i].SetLED(frc::Color::kOrangeRed);
    //     }
    // } else if(Robot::GetRobot()->m_UnderBotLaser.Get() == 0) {
    //     for(int i = 0; i < m_LEDMiddle.size(); i++) {
    //         m_LEDMiddle[i].SetLED(frc::Color::kHotPink);
    //     }
    // } else {
    //     SponsorBoardAllianceColor();
    // }
    // if(Robot::GetRobot()->GetVision().GetLimeLight()->GetNumber("tv", 0) == 1) {
    //     for(int i = numLEDs/2; i < numLEDs; i++) {
    //         m_LEDMiddle[i].SetLED(frc::Color::kDarkGreen);
    //     }
    // }
    if (Robot::GetRobot()->GetVision().GetLimeLight()->GetNumber("tv", 0) == 1) {
        for (int i = 0; i < numLEDsLeftR1; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
        for(int i = numLEDsLeftEnd; i < numLEDsFrontR1; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
        for(int i = numLEDsFrontEnd; i <= numLEDsRightR1; i++) {
            m_LEDBuffer[i].SetLED(darkGreen );
        }
    } else if (Robot::GetRobot()->GetDriveTrain().LockOnStatus == true) {
        for (int i = 0; i < numLEDsLeftR1; i++) {
            m_LEDBuffer[i].SetLED(yellow);
        }
        for(int i = numLEDsLeftEnd; i < numLEDsFrontR1; i++) {
            m_LEDBuffer[i].SetLED(yellow);
        }
        for(int i = numLEDsFrontEnd; i <= numLEDsRightR1; i++) {
            m_LEDBuffer[i].SetLED(yellow);
        }
    }
    
    if (Robot::GetRobot()->m_DustpanLaser.Get() == 0) {
        for (int i = numLEDsLeftR1; i < numLEDsLeftR2; i++) {
            m_LEDBuffer[i].SetLED(orangeRed);
        }
        for(int i = numLEDsFrontR1; i < numLEDsFrontR2; i++) {
            m_LEDBuffer[i].SetLED(orangeRed);
        }
        for(int i = numLEDsRightR1; i <= numLEDsRightR2; i++) {
            m_LEDBuffer[i].SetLED(orangeRed);
        }
    } else if (Robot::GetRobot()->m_UnderBotLaser.Get() == 0) {
        for (int i = numLEDsLeftR1; i < numLEDsLeftR2; i++) {
            m_LEDBuffer[i].SetLED(darkViolet);
        }
        for(int i = numLEDsFrontR1; i < numLEDsFrontR2; i++) {
            m_LEDBuffer[i].SetLED(darkViolet);
        }
        for(int i = numLEDsRightR1; i < numLEDsRightR2; i++) {
            m_LEDBuffer[i].SetLED(darkViolet);
        }
    }

    if(abs(Robot::GetRobot()->GetArm().GetShooterMotor1().GetVelocity().GetValueAsDouble()) > 70) {
        for (int i = numLEDsLeftR2; i <= numLEDsLeftEnd; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
        for(int i = numLEDsFrontR2; i <= numLEDsFrontEnd; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
        for(int i = numLEDsRightR2; i <= numLEDsRightEnd; i++) {
            m_LEDBuffer[i].SetLED(darkGreen);
        }
    }
}

void LED::PickupFlashing() {
    if(m_FlashCounter == 20) {
        for(int i = 0; i < m_LEDBuffer.size(); i++) {
            m_LEDBuffer[i].SetLED(white);
        }
    } else if(m_FlashCounter == 40) {
        m_FlashCounter = 0;
        for(int i = 0; i < m_LEDBuffer.size(); i++) {
            m_LEDBuffer[i].SetRGB(0,0,0);
        }
    } else {
        m_FlashCounter++;
    }
}

// void LED::VisionCanSee() {
//     if(Robot::GetRobot()->GetVision().GetLimeLight()->GetNumber("tv", 0.0) == 1) {
//         if(Robot::GetRobot()->m_DustpanLaser.Get() == 0) {
//         for(int i = 0; i < numLEDs; i += 2)
//             m_LEDBuffer[i].SetLED(frc::Color::kGreen);
//         for(int i = 1; i < numLEDs; i += 2)
//             m_LEDBuffer[i].SetLED(frc::Color::kGreen);
//     } else if(Robot::GetRobot()->m_UnderBotLaser.Get() == 0) {
//         for(int i = 0; i < numLEDs; i++) {
//             m_LEDBuffer[i].SetLED(frc::Color::kPurple);
//         }
