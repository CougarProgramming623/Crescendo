// #pragma once

// #include <frc/AddressableLED.h>
// #include <frc/util/Color.h>
// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/button/JoystickButton.h>

// //#include "Util.h"

// class LED{
    
//     public:
    
//         LED();
//         void Init();
// // 
//         void LowBattery();

//         void SetData();

//         void EndGame();

//         void SponsorBoardAllianceColor();
//         void SponsorBoardSolid(frc::Color allianceColor);
//         void SponsorBoardSolid(int R, int G, int B);

//         void SponsorBoardRainbow();

//         void SponsorBoardFlash(frc::Color allianceColor);     
//         void SponsorBoardFlash(int R, int G, int B);     

//         void EyesAllianceColor();
//         void EyesSolidYellow(frc::Color);
//         void EyesSolidPurple(frc::Color);
//         void EyesSolid(frc::Color allianceColor);
//         void EyesSolid(int R, int G, int B);

//         void EyesAngry();
//         void EyesSleepy();
//         void EyeRoll();

//         bool m_IsTele;


//     private:

//         frc::AddressableLED m_AddressableLED{9};
//         std::array <frc::AddressableLED::LEDData, 180> m_LEDBuffer;
//         int m_IterationTracker;

//         frc::Color m_AllianceColor;

//         frc2::Trigger m_EyesYellow;
//         frc2::Trigger m_EyesPurple;
//         frc2::Trigger m_EyesWhite;
// };