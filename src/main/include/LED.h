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
        void PickupFlashing();
        void SponsorBoardRGB();

        bool m_IsTele;
        int numLEDsTotal = 67;//181;
        // int numLEDsLeftR1 = 19;
        // int numLEDsLeftR2 = 38;
        // int numLEDsLeftEnd = 57;
        int numLEDsFrontR1 = 22;//79;
        int numLEDsFrontR2 = 45;//102;
        int numLEDsFrontEnd = 67;//124;
        // int numLEDsRightR1 = 143;
        // int numLEDsRightR2 = 162;
        // int numLEDsRightEnd = numLEDsTotal; //181
        std::array <frc::AddressableLED::LEDData, 67> m_LEDBuffer; //181
    private:

        frc::AddressableLED m_AddressableLED{2};
        int m_FlashCounter;

        frc::Color m_AllianceColor;

        frc2::Trigger m_EyesYellow;
        frc2::Trigger m_EyesPurple;
        frc2::Trigger m_ButtonLeds;

        frc::Color red = frc::Color(127, 0, 0);
        frc::Color blue = frc::Color(0, 0, 127);
        frc::Color white = frc::Color(127, 127, 127);
        frc::Color darkGreen = frc::Color(0, 50, 0);
        frc::Color yellow = frc::Color(127, 127, 0);
        frc::Color orangeRed = frc::Color(127, 34, 0);
        frc::Color darkViolet = frc::Color(74, 0, 105);

        bool LockOnStatus;
        int m_IterationTracker;
};