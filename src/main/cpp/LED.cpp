#include "LED.h"
#include "Util.h"
#include "Constants.h"
#include "Robot.h"
#include "frc/RobotBase.h"
#include "frc/Timer.h"

//set values
const int kSTART_BL = 0;
const int kSTART_TL = 0;
const int kSTART_TR = 0;
const int kSTART_BR = 0;

const int kSTART_EYE_1  = 0;
const int kEND_EYE_1    = 0;
const int kSTART_EYE_2  = 0;
const int kEND_EYE_2    = 0;
const int kNUM_LED = -1;


const int kNum_LED = 180; // 28 for the eyes, 31 on each base of the board + 7 on each side of the board {[(31 + 7)*2]*2}

frc::Color colorArray[] = {frc::Color::kRed, frc::Color::kYellow, frc::Color::kGreen, frc::Color::kBlue, frc::Color::kPurple};
frc::Color redWhiteArray[] = {frc::Color::kWhite, frc::Color::kRed};

LED::LED()  :
    m_EyesYellow(frc2::Button(BUTTON_L(15))),
    m_EyesPurple(frc2::Button(BUTTON_L(16))),
    m_EyesWhite([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(1);})
{}

void LED::Init(){
    DebugOutF("LED Init");
    m_AddressableLED.SetLength(kNum_LED);
    m_AddressableLED.Start();
    m_IterationTracker = 0;
    m_IsTele = false;
    SponsorBoardAllianceColor();
    EyesAllianceColor();

    m_EyesYellow.WhenPressed(new frc2::InstantCommand([&]{
            //DebugOutF("cone");
            EyesSolid(frc::Color::kYellow);
            SponsorBoardSolid(frc::Color::kYellow);
            SetData();
        })
    );

    m_EyesPurple.WhenPressed(new frc2::InstantCommand([&]{
            //DebugOutF("cube");
            EyesSolid(frc::Color::kPurple);
            SponsorBoardSolid(frc::Color::kPurple);
            SetData();
        })
    );

    m_EyesWhite.WhenPressed(new frc2::InstantCommand([&]{
            //DebugOutF("white");
            EyesSolid(frc::Color::kWhite);
            SponsorBoardSolid(frc::Color::kWhite);
            SetData();

        })
    );
} 

void LED::SetData(){ m_AddressableLED.SetData(m_LEDBuffer); }

void LED::LowBattery(){
    if (false){ //FIX if we want low battery
        for(int i = 0; i <= kNum_LED - 28; i++){
            for(int j = (i*4) + m_IterationTracker; j < ((i+1)*4) + m_IterationTracker; j++){
                m_LEDBuffer[j % kNum_LED].SetLED(frc::Color::kGreen);
                m_LEDBuffer[j + 4 % kNum_LED].SetLED(frc::Color::kRed);
            }
    }
    
    m_IterationTracker++;
    if (m_IterationTracker == kNum_LED){
        m_IterationTracker = 0;
    }
    }
    
}

void LED::EndGame(){
    if ((int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) <= 30 && (int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) > 28 
        && (int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) != -1){
        SponsorBoardFlash(frc::Color::kWhite);
    } else if ((int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) <= 20
        && (int)COB_GET_ENTRY(COB_KEY_MATCHTIME).GetDouble(31) != -1){
        SponsorBoardFlash(frc::Color::kWhite);
    }
}

// m_NavXReset.WhenPressed(
//     new frc2::InstantCommand([&]{
//       DebugOutF("NavX Zero");
//       zeroGyroscope();
//     })
//   );


void LED::EyesAllianceColor(){
    if (!Robot::GetRobot()->GetButtonBoard().GetRawButton(15)
        && !Robot::GetRobot()->GetButtonBoard().GetRawButton(16)
        && !Robot::GetRobot()->GetJoyStick().GetRawButton(1)){
        EyesSolid(m_AllianceColor);
    } 
}

void LED::SponsorBoardAllianceColor(){
    if (!Robot::GetRobot()->GetButtonBoard().GetRawButton(15)
        && !Robot::GetRobot()->GetButtonBoard().GetRawButton(16)
        && !Robot::GetRobot()->GetJoyStick().GetRawButton(1)){
        if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
            m_AllianceColor = frc::Color::kRed;
        } else {
            m_AllianceColor = frc::Color::kBlue;
        }
        if (frc::DriverStation::GetAlliance() != frc::DriverStation::Alliance::kRed &&
            frc::DriverStation::GetAlliance() != frc::DriverStation::Alliance::kBlue) {
            m_AllianceColor = frc::Color::kWhite;
        }
        // if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
        //     m_AllianceColor = frc::Color::kRed;
        // } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        //     m_AllianceColor = frc::Color::kBlue;
        // } else {
        //     m_AllianceColor = frc::Color::kWhite;
        // }
        SponsorBoardSolid(m_AllianceColor);
    }
}

void LED::SponsorBoardSolid(frc::Color color){
    for(int i = 0; i < kNum_LED - 29; i++){
            m_LEDBuffer[i].SetLED(color);
    }
}

void LED::SponsorBoardSolid(int R, int G, int B){
    for(int i = 0; i < kNum_LED - 29; i++){
        m_LEDBuffer[i].SetRGB(R, G, B);
    }
}

/*
Start from 255 0 0, then count up g to 255 255 0, then count down red to 0 255 0,
then count up blue to 0 255 255, then count down green to 0 0 255,
then count up red to 255 0 255, then count down blue to 255 0 0.
*/
void LED::SponsorBoardRainbow(){
    for (int k = 0; k < 5; k++){
        if (Robot::GetRobot()->m_COBTicks % 5 == 0){
            SponsorBoardSolid(colorArray[k]);
        }
    }
}

void LED::SponsorBoardFlash(frc::Color color){
    if (LED::m_IsTele){
        if (std::fmod(frc::Timer::GetMatchTime().to<double>(), 1) < 0.5){
            SponsorBoardSolid(m_AllianceColor);
        } else {
            SponsorBoardSolid(0,0,0);
        }
    }
}

//frc::Timer::GetMatchTime().to<double>() <= 30 && frc::Timer::GetMatchTime().to<double>() > 28

void LED::SponsorBoardFlash(int R, int G, int B){
    if(m_IterationTracker % 2 == 0){
        SponsorBoardSolid(R, G, B);
    } else {
        SponsorBoardSolid(0, 0, 0);
    }
}

// void LED::EyesSolid(frc::Color){
//     for(int i = kSTART_EYE_1; i < kEND_EYE_1; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kYellow);
//     for(int i = kSTART_EYE_2; i < kEND_EYE_2; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kYellow);
// }
// void LED::EyesSolidPurple(frc::Color){
//     for(int i = kSTART_EYE_1; i < kEND_EYE_1; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kPurple);
//     for(int i = kSTART_EYE_2; i < kEND_EYE_2; i++)
//         m_LEDBuffer[i].SetLED(frc::Color::kPurple);
// }

void LED::EyesSolid(frc::Color color){
    for(int i = kNum_LED - 1; i >= kNum_LED - 28; i--){
        m_LEDBuffer[i].SetLED(color);
    }
}

void LED::EyesSolid(int R, int G, int B){
    for(int i = kNum_LED - 1; i >= kNum_LED - 28; i--){
        m_LEDBuffer[i].SetRGB(R, G, B);
    }
}

void LED::EyesAngry(){}
void LED::EyesSleepy(){}
void LED::EyeRoll(){}
