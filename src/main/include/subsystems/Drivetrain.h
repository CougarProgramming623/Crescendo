#pragma once

//allows us to use sysid::Direction - the direction for the motor during a sysid routine
#include <functional>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/geometry/Rotation2d.h>


//for sysid
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/RobotController.h>

//copied includes
#include <frc/geometry/Transform2d.h>
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
//#include <./commands/TrajectoryCommand.h>
//#include <./commands/DriveToPosCommand.h>
#include <frc/Timer.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/Command.h>
#include <unordered_map>

//using ctre::phoenix::motorcontrol::can::TalonFX;

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void BaseDrive(frc::ChassisSpeeds chassisSpeeds);
  void DriveInit();
  void BreakMode(bool on);
  void Periodic() override;

  frc::Translation2d m_FrontLeftLocation;
  frc::Translation2d m_FrontRightLocation;
  frc::Translation2d m_BackLeftLocation;
  frc::Translation2d m_BackRightLocation;
  
  bool m_DriveToPoseFlag = false;

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  inline frc::SwerveDriveKinematics<4> GetKinematics() { return m_Kinematics; }
  inline frc::SwerveDrivePoseEstimator<4>* GetOdometry(){ return &m_Odometry; }
  inline frc::HolonomicDriveController GetHolonomicController(){ return m_HolonomicController; }

  inline std::array<frc::SwerveModulePosition, 4> GetModulePositions(){ return m_ModulePositions; }

  // pathplanner::FollowPathWithEvents* TruePath();
  // pathplanner::FollowPathWithEvents* TrueAuto(PathPlannerTrajectory traj);

  inline bool GetIsBalancing() { return m_IsBalancing; }
  inline void SetIsBalancing(bool b) { m_IsBalancing = b; }

  frc2::FunctionalCommand AutoBalanceCommand();
  void AutoBalanceFunction();

  //TrajectoryCommand DriveToPos(frc::Pose2d target);


//how fast the robot should be able to drive
  const units::meters_per_second_t kMAX_VELOCITY_METERS_PER_SECOND = units::meters_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI);

  std::array<frc::SwerveModulePosition, 4> m_ModulePositions;

  const double kMAX_VOLTAGE = 12.0; //FIX
  
  SwerveModule m_FrontLeftModule;
  SwerveModule m_FrontRightModule;
  SwerveModule m_BackLeftModule;
  SwerveModule m_BackRightModule;

  //theoretical maximum angular velocity - can be replaced with measure amount
  const units::radians_per_second_t kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = units::radians_per_second_t(6380.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * M_PI / std::sqrt(Pow((DRIVETRAIN_TRACKWIDTH_METERS / 2), 2) + Pow((DRIVETRAIN_WHEELBASE_METERS / 2), 2)));

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_EventMap;

  int m_SelectedGrid;

  frc::Pose2d m_PoseMatrix[3][3] = {
    {TLPOSE, TCPOSE, TRPOSE},
    {MLPOSE, MCPOSE, MRPOSE},
    {BLPOSE, BCPOSE, BRPOSE},
  };

  frc::Pose2d m_TransformedPose;
  
  int m_VisionCounter;
  frc::Pose2d m_VisionRelative;

  frc2::Trigger m_JoystickOuttake;


  private:

  frc2::sysid::SysIdRoutine m_SysIdRoutine{
    frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            m_FrontLeftModule.m_DriveController.SetReferenceVoltage(driveVoltage.value());
            m_FrontRightModule.m_DriveController.SetReferenceVoltage(driveVoltage.value());
            m_BackLeftModule.m_DriveController.SetReferenceVoltage(driveVoltage.value());
            m_BackRightModule.m_DriveController.SetReferenceVoltage(driveVoltage.value());
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            // log->Motor("drive-frontleft")
            //     .voltage(units::voltage::volt_t(m_FrontLeftModule.m_DriveController.GetMotor().GetMotorVoltage().GetValueAsDouble() * frc::RobotController::GetBatteryVoltage().value()))
            //     .position(units::meter_t{m_FrontLeftModule.GetPosition().distance()})
            //     .velocity(units::meters_per_second_t{m_FrontLeftModule.GetDriveVelocity()});
            // log->Motor("drive-frontright")
            //     .voltage(units::voltage::volt_t(m_FrontRightModule.m_DriveController.GetMotor().GetMotorVoltage().GetValueAsDouble() * frc::RobotController::GetBatteryVoltage().value()))
            //     .position(units::meter_t{m_FrontRightModule.GetPosition().distance()})
            //     .velocity(units::meters_per_second_t{m_FrontRightModule.GetDriveVelocity()});
            log->Motor("drive-backleft")
                .voltage(units::voltage::volt_t(m_BackLeftModule.m_DriveController.GetMotor().GetMotorVoltage().GetValueAsDouble() * frc::RobotController::GetBatteryVoltage().value()))
                .position(units::meter_t{m_BackLeftModule.GetPosition().distance()})
                .velocity(units::meters_per_second_t{m_BackLeftModule.GetDriveVelocity()});
            // log->Motor("drive-backright")
            //     .voltage(units::voltage::volt_t(m_BackRightModule.m_DriveController.GetMotor().GetMotorVoltage().GetValueAsDouble() * frc::RobotController::GetBatteryVoltage().value()))
            //     .position(units::meter_t{m_BackRightModule.GetPosition().distance()})
            //     .velocity(units::meters_per_second_t{m_BackRightModule.GetDriveVelocity()});
          },
          this
      }
  };
  
  frc::Timer m_Timer;

  frc2::Trigger m_TestJoystickButton;
  frc2::Trigger m_JoystickButtonTwo;
  frc2::Trigger m_ExtraButton;
  frc2::Trigger m_NavXResetButton;
  frc2::Trigger m_ExtraJoystickButton;

  bool m_IsBalancing;

  frc::SwerveDriveKinematics<4> m_Kinematics;
  frc::SwerveDrivePoseEstimator<4> m_Odometry;
  
  frc::Rotation2d m_Rotation;             
  frc::ChassisSpeeds m_ChassisSpeeds;

  std::array<frc::SwerveModuleState, 4> m_ModuleStates;
  
  frc::PIDController m_xController;
  frc::PIDController m_yController;
  frc::ProfiledPIDController <units::radians> m_ThetaController;
  frc::HolonomicDriveController m_HolonomicController;

  
};