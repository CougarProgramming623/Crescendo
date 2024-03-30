#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>

//#include "Util.h"

class LED {
    
    public:
    
        LED();
        void Init();
        void SetData();
        void SponsorBoardAllianceColor();
        void SponsorBoardSolid(frc::Color allianceColor);
        void LaserSensors();
        void VisionCanSee();

        // void LowBattery();
        // void EndGame();
        // void SponsorBoardSolid(int R, int G, int B);
        // void SponsorBoardRainbow();
        // void SponsorBoardFlash(frc::Color allianceColor);     
        // void SponsorBoardFlash(int R, int G, int B);     
        // void EyesAllianceColor();
        // void EyesSolidYellow(frc::Color);
        // void EyesSolidPurple(frc::Color);
        // void EyesSolid(frc::Color allianceColor);
        // void EyesSolid(int R, int G, int B);
        // void EyesAngry();
        // void EyesSleepy();
        // void EyeRoll();

        bool m_IsTele;
        int numLEDs = 67;
    private:

        frc::AddressableLED m_AddressableLED{2};
        std::array <frc::AddressableLED::LEDData, 12> m_LEDBuffer;
        int m_IterationTracker;

        frc::Color m_AllianceColor;

        frc2::Trigger m_EyesYellow;
        frc2::Trigger m_EyesPurple;
        frc2::Trigger m_ButtonLeds;
};