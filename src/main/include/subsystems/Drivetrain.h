#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/SerialPort.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/geometry/Rotation2d.h>


//copied includes
#include <frc/geometry/Transform2d.h>
#include <frc/Servo.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/HolonomicDriveController.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include <array>
#include <fstream>
#include "./Util.h"
#include "Constants.h"
#include "SwerveModule.h"
#include <frc2/command/SubsystemBase.h>
#include "commands/DriveWithJoystick.h"
#include <frc/Timer.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/commands/FollowPathCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/Command.h>
#include <unordered_map>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc/drive/DifferentialDrive.h>

using namespace frc;

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();

  void BaseDrive(frc::ChassisSpeeds chassisSpeeds);
  void DriveInit();
  void BrakeMode(bool on);
  void Periodic() override;

  frc::Translation2d m_FrontLeftLocation;
  frc::Translation2d m_FrontRightLocation;
  frc::Translation2d m_BackLeftLocation;
  frc::Translation2d m_BackRightLocation;
  
  bool m_DriveToPoseFlag = false;

  inline frc::SwerveDriveKinematics<4> GetKinematics() { return m_Kinematics; }
  inline frc::SwerveDriveOdometry<4>* GetOdometry(){ return & m_Odometry; }
  inline frc::HolonomicDriveController GetHolonomicController(){ return m_HolonomicController; }
  inline hardware::TalonFX& GetClimbMotor() {return m_Climb;}
  inline frc::Timer& GetTimer() {return m_Timer;}

  inline void SetChassisSpeeds(frc::ChassisSpeeds speeds) {m_ChassisSpeeds = speeds;}

  inline std::array<frc::SwerveModulePosition, 4> GetModulePositions(){ return m_ModulePositions; }

  wpi::array<frc::SwerveModuleState, 4> getStates();
  Pose2d getPose();
  void resetPose(Pose2d pose);
  ChassisSpeeds getRobotRelativeSpeeds();
  void DriveRobotRelative(ChassisSpeeds robotRelativeSpeeds);
  void SetStates(wpi::array<frc::SwerveModuleState, 4> states);

  // wpi::array<frc::SwerveModuleState, 4> getStates();

  inline bool GetIsBalancing() { return m_IsBalancing; }
  inline void SetIsBalancing(bool b) { m_IsBalancing = b; }

  frc2::FunctionalCommand AutoBalanceCommand();
  double getStandardDeviation(std::vector<double> arr);


  //how fast the robot should be able to drive
  const units::meters_per_second_t kMAX_VELOCITY_METERS_PER_SECOND = units::meters_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI); //4.952 m/s

  std::array<frc::SwerveModulePosition, 4> m_ModulePositions;

  const double kMAX_VOLTAGE = 12.0; //FIX
  
  SwerveModule m_FrontLeftModule;
  SwerveModule m_FrontRightModule;
  SwerveModule m_BackLeftModule;
  SwerveModule m_BackRightModule;
  //hardware::TalonFX m_ShooterMotor1;
  //hardware::TalonFX m_ShooterMotor2;

  SwerveModuleState m_FrontLeftState;
  SwerveModuleState m_FrontRightState;
  SwerveModuleState m_BackLeftState;
  SwerveModuleState m_BackRightState;

  //theoretical maximum angular velocity - can be replaced with measure amount
  const units::radians_per_second_t kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = units::radians_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI / std::sqrt(Pow((DRIVETRAIN_TRACKWIDTH_METERS / 2), 2) + Pow((DRIVETRAIN_WHEELBASE_METERS / 2), 2)));

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_EventMap;

  frc::Pose2d m_VisionPos;

  std::vector<double> m_VisionPosXArray;
  std::vector<double> m_VisionPosYArray;
  std::vector<double> m_VisionPosTArray;

  double xSum;
  double xMean;
  double xSD;
  double ySum;
  double yMean;
  double ySD;
  double tSum;
  double tMean;
  double tSD;

  int m_SelectedGrid;

  int driveState;

  // frc::Pose2d m_PoseMatrix[3][3] = {
  //   {TLPOSE, TCPOSE, TRPOSE},
  //   {MLPOSE, MCPOSE, MRPOSE},
  //   {BLPOSE, BCPOSE, BRPOSE},
  // };

  frc::Pose2d m_TransformedPose;
  
  int m_VisionCounter;
  frc::Pose2d m_VisionRelative;

  bool LockOnStatus;
  
  private:

  //motors
  hardware::TalonFX m_Climb;
  
  frc::Timer m_Timer;

  //triggers
  frc2::Trigger m_ClimbRobotUp;
	frc2::Trigger m_ClimbRobotDown;
  frc2::Trigger m_VisionAim;
  frc2::Trigger m_StrafeLeft;
  frc2::Trigger m_StrafeRight;
  frc2::Trigger m_Lock180Button;
  frc2::Trigger m_LockOnButton;
  frc2::Trigger m_JoystickFlywheel;
  frc2::Trigger m_NavXResetButton;

  bool m_IsBalancing;

  frc::SwerveDriveKinematics<4> m_Kinematics;
  frc::SwerveDriveOdometry<4> m_Odometry;
  
  frc::Rotation2d m_Rotation;             
  frc::ChassisSpeeds m_ChassisSpeeds;

  std::array<frc::SwerveModuleState, 4> m_ModuleStates;
  
  frc::PIDController m_xController;
  frc::PIDController m_yController;
  frc::ProfiledPIDController <units::radians> m_ThetaController;
  frc::HolonomicDriveController m_HolonomicController;
};