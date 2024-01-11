// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/SerialPort.h>
#include <frc2/command/button/Trigger.h>


#include <frc2/command/Command.h>

#include <pathplanner/lib/PathPlannerPath.h>
#include "LED.h"
#include "subsystems/DriveTrain.h"
#include <AHRS.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "COB.h"
#include "Vision.h"
#include "subsystems/Arm.h"
#include "subsystems/MotionMagicTest.h"
#include "LED.h"
#include <frc/geometry/Pose2d.h>
#include "./subsystems/Intake.h"


class Robot : public frc::TimedRobot {
 public:
  Robot();
  static inline Robot* GetRobot() { return s_Instance; }
  inline Arm& GetArm() { return m_Arm; }
  inline frc::Joystick& GetButtonBoard() { return m_ButtonBoard; }
  inline frc::Joystick& GetButtonBoardTwo() { return m_ButtonBoardTwo; }
  inline frc::Joystick& GetJoystick() { return m_Joystick; }

  void RobotInit() override;
  void AutoButtons();
  frc::Pose2d TransformPose(frc::Pose2d SelectedPose);
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  inline AHRS& GetNavX() { return m_NavX; }
  inline void zeroGyroscope() {m_NavX.ZeroYaw();}
  inline double getYaw() {return m_NavX.GetYaw();}
  inline double getPitch() {return m_NavX.GetPitch();}

  inline DriveTrain& GetDriveTrain() { return m_DriveTrain; }
  inline frc::Joystick& GetJoyStick() { return m_Joystick; }

  double GetAngle() {return fmod(360 - GetNavX().GetYaw(), 360); }
  
  inline COB& GetCOB() { return m_COB; }
  inline Vision& GetVision() { return m_Vision; }


  double previousErrorX = 0;
  double previousErrorY = 0;
  double dErrorY = 0;
  double previousErrorT = 0;
  
  double previousValueX = 0;
  double previousValueY = 0;
  double previousValueT = 0;

  frc2::Button m_TL;
	frc2::Button m_TC;
	frc2::Button m_TR;
	frc2::Button m_ML;
	frc2::Button m_MC;
	frc2::Button m_MR;
	frc2::Button m_BL;
	frc2::Button m_BC;
	frc2::Button m_BR;

	frc2::Button m_LeftGrid;
	frc2::Button m_CenterGrid;
	frc2::Button m_RightGrid;

  frc2::Button m_BigRed;
  frc2::Button m_GroundPickup;

  frc2::Button m_SingleSub;
  frc2::Button m_SingleSubCube;
  frc2::Button m_DoubleSub;

  frc2::Button m_MidCone;
  frc2::Button m_MidCube;
  frc2::Button m_PlacingMode;

  frc2::Button m_NavXReset;
  frc2::Button m_AutoBalance;
  frc2::Button m_VisionPoseReset;

  frc2::Button m_Print;
  int m_COBTicks;


  int SelectedRow;
	int SelectedColumn;

  Intake m_Intake;

  bool m_AutoFlag;
  int m_ColOffset;

 private:

  frc2::ParallelCommandGroup* m_ArmCommand;

  static Robot* s_Instance;

  AHRS m_NavX;

  frc::Joystick m_Joystick = frc::Joystick(1);

  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;


  frc2::Button m_LEDYellow;
  frc2::Button m_LEDPurple;
  LED m_LED;

  Arm m_Arm;

  frc::Timer m_AutoTimer;
  DriveTrain m_DriveTrain;

  Vision m_Vision;

  COB m_COB;
  frc::Joystick m_ButtonBoard = frc::Joystick(0);
  frc::Joystick m_ButtonBoardTwo = frc::Joystick(2);

  std::string m_AutoPath;

  MotionMagicTest m_MMT;

};
