#pragma once

#include <frc2/command/PrintCommand.h>


inline void DebugOutF(const std::string_view message) {
  frc2::CommandScheduler::GetInstance().Schedule(new frc2::PrintCommand(message));
}

inline double Deg2Rad(double deg) {
    return deg * (M_PI/180);
}

inline double Rad2Deg(double rad) {
    return rad * (180/M_PI);
}

inline long double Pow(double n, int pow){
  double res = n;
  for (int i = 1; i < pow; i++) {
    res = res * n;
  }
  return res;
}

#define BUTTON_L(id) \
  [&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(id); }

  #define BUTTON_L_TWO(ID) \
  [&] { return Robot::GetRobot()->GetButtonBoardTwo().GetRawButton(ID); }
