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

        bool m_IsTele;
        int numLEDs = 67;
        std::array <frc::AddressableLED::LEDData, 67> m_LEDMiddle;
        std::array <frc::AddressableLED::LEDData, 55> m_LEDLeft;
        // std::array <frc::AddressableLED::LEDData, 57> m_LEDRight;
    private:

        frc::AddressableLED m_AddressableLED{2};
        int m_FlashCounter;

        frc::Color m_AllianceColor;

        frc2::Trigger m_EyesYellow;
        frc2::Trigger m_EyesPurple;
        frc2::Trigger m_ButtonLeds;

        bool LockOnStatus;

        int sect1left = 17;
        int sect2left = 35;
        int sect3left = 54;

        int sect1right = 17;
        int sect2right = 35;
        int sect3right = 54;

        int sect1middle = 21;
        int sect2middle = 42;
        int sect3middle = 66;
};