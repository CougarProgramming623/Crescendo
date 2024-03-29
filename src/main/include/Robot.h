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

#include <pathplanner/lib/path/PathPlannerPath.h>
#include "LED.h"
#include "subsystems/DriveTrain.h"
#include <AHRS.h>
#include <frc/Joystick.h>
#include <frc2/command/button/Trigger.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "COB.h"
#include "Vision.h"
#include "subsystems/Arm.h"
#include "LED.h"
#include <commands/Flywheel.h>
#include <commands/Shoot.h>
#include <commands/Intake.h>
#include <frc/geometry/Pose2d.h>
//#include "./subsystems/Intake.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

#include <frc/Timer.h>


class Robot : public frc::TimedRobot {
 public:
  Robot();
  static inline Robot* GetRobot() { return s_Instance; }
  inline Arm& GetArm() { return m_Arm; }
  inline PivotToPos& GetPivotPos() {return m_PivotToPos;}
  inline frc::GenericHID& GetButtonBoard() { return m_ButtonBoard; }
  inline frc::GenericHID& GetJoystick() { return m_Joystick; }
  inline frc::GenericHID& GetButtonBoardTwo() { return m_ButtonBoardTwo; }

  void RobotInit() override;
  void AutoButtons();
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

  pathplanner::PathPlannerTrajectory getTrajectory();
  frc::Pose2d TransformPose(frc::Pose2d SelectedPose);
  void MotorInversionCheck();
  // void MotorInversionCorrection(ctre::phoenix6::hardware::TalonFX motor, int ID, bool invert);

  //motor control requests - LOOK, VERY IMPORTANT
  //controls::VoltageOut m_VoltageOutRequest{0_V};
  controls::PositionDutyCycle m_PositionDutyCycle{units::angle::turn_t(0)};
  controls::DutyCycleOut m_DutyCycleOutRequest{0};
  controls::VoltageOut m_VoltageOutRequest{0_V};
  controls::MotionMagicDutyCycle m_MotionMagicRequest{units::angle::turn_t(0)};

  bool inversionPrint = true;

  double previousErrorX = 0;
  double previousErrorY = 0;
  double dErrorY = 0;
  double previousErrorT = 0;
  
  double previousValueX = 0;
  double previousValueY = 0;
  double previousValueT = 0;

  //BUTTONBOARD
  frc2::Trigger m_VisionPoseReset;

  frc2::Trigger m_Print;
  frc2::Trigger m_Print2;
  frc2::Trigger m_Print3;
  frc2::Trigger m_Print4;

  int m_COBTicks;
  //double m_Set;

  int SelectedRow;
	int SelectedColumn;

  //Intake m_Intake;

  bool m_AutoFlag;
  int m_ColOffset;

 private:
  
  bool flash;
  
  frc::Pose2d startingPose;

  frc2::ParallelCommandGroup* m_ArmCommand;

  static Robot* s_Instance;

  AHRS m_NavX;

  frc::Joystick m_ButtonBoard = frc::Joystick(0);
  frc::Joystick m_Joystick = frc::Joystick(1);
  frc::Joystick m_ButtonBoardTwo = frc::Joystick(2);

  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;


  frc2::Trigger m_LEDYellow;
  frc2::Trigger m_LEDPurple;
  LED m_LED;

  Arm m_Arm;

  PivotToPos m_PivotToPos;

  frc::Timer m_AutoTimer;
  DriveTrain m_DriveTrain;//Drivetrain "Master" object to access all instances(objects) of the drivetrain class
  //Shooter m_Shooter;//Shooter "Master" object to access all instances(objects) of the shooter class
  //Intake m_Intake;//Intake "Master" object to access all instances(objects) of the intake class

  Vision m_Vision;

  COB m_COB;
  

  std::string m_AutoPath;

  //MotionMagicTest m_MMT;

};