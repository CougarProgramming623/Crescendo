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
        std::array <frc::AddressableLED::LEDData, 67> m_LEDBuffer;
    private:

        frc::AddressableLED m_AddressableLED{2};
        int m_FlashCounter;

        frc::Color m_AllianceColor;

        frc2::Trigger m_EyesYellow;
        frc2::Trigger m_EyesPurple;
        frc2::Trigger m_ButtonLeds;

        bool LockOnStatus;
};