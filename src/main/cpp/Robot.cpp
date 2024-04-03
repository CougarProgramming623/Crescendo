// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include "Util.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/RobotController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandPtr.h>
#include "commands/LockOn.h"
#include <frc/DriverStation.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include "commands/ConstantPivot.h"
#include <commands/AutoTest.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/PrintCommand.h>
#include "commands/TrajectoryCommand.h"

#include <chrono>


using ctre::phoenix::motorcontrol::ControlMode;
using namespace pathplanner;
using namespace std::chrono;

Robot* Robot::s_Instance = nullptr;

Robot::Robot() :
m_NavX(frc::SerialPort::Port(2), AHRS::SerialDataType(0), uint8_t(66)),
m_LED(),
m_PivotToPos(420)
{
  // DebugOutF("inside robot constructor");
  s_Instance = this;
}


void Robot::RobotInit() {
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  s_Instance = this;
  // DebugOutF("initalizing drivetrain w/ motors");
  m_DriveTrain.DriveInit();
  m_Arm.ArmInit();
  
  // DebugOutF("initalizing motors finished");
  // DebugOutF("x: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
  // DebugOutF("y: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
  // DebugOutF("theta: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));

  DebugOutF("BL Voltage: " + std::to_string(GetDriveTrain().m_BackLeftModule.GetSteerSensorVoltage()));
  DebugOutF("BR Voltage: " + std::to_string(GetDriveTrain().m_BackRightModule.GetSteerSensorVoltage()));
  DebugOutF("FL Voltage: " + std::to_string(GetDriveTrain().m_FrontLeftModule.GetSteerSensorVoltage()));
  DebugOutF("FR Voltage: " + std::to_string(GetDriveTrain().m_FrontRightModule.GetSteerSensorVoltage()));
  DebugOutF("Max Sensor Voltage: " + std::to_string(frc::RobotController::GetVoltage5V()));

  NamedCommands::registerCommand("Flywheel", std::move(Flywheel().ToPtr()));
  NamedCommands::registerCommand("Shoot", std::move(Shoot().ToPtr()));
  
  AutoButtons();
  m_LED.Init();
  
  if(m_Print.Get()) {
    m_AutoPath += "left";
    m_AngleOffset = 120;
  } else if(m_Print2.Get()) {
    m_AutoPath += "right";
    m_AngleOffset = 240;
  } else {
    m_AutoPath += "middle";
    m_AngleOffset = 180;
  }

  if(DriverStation::GetAlliance() == DriverStation::Alliance::kBlue) {
    m_AutoPath += "Blue";
  } else {
    m_AutoPath += "Red";
  }
  
  m_COBTicks = 0;
  m_ArmCommand = nullptr;
}


void Robot::AutoButtons() {
  //BUTTONBOARD
  m_Print = frc2::Trigger(BUTTON_L(16));
  m_Print2 = frc2::Trigger(BUTTON_L(15));
  m_Print3 = frc2::Trigger(BUTTON_L(14));
  m_Print4 = frc2::Trigger(BUTTON_L(13));
  m_FlashLEDs  = frc2::Trigger(BUTTON_L(AIM_BUTTON));

  m_Print.WhileTrue(new frc2::InstantCommand([&] {
    // DebugOutF("Stringpot Value: " + std::to_string(GetArm().GetStringPot().GetAverageValue()));
    // Vision vision = GetVision();
    // if(vision.GetLimeLight()->GetNumber("tv", 0.0) == 1) {
    //   int id = vision.GetLimeLight()->GetNumber("tid", 0.0);
    //   double theta = vision.GetLimeLight()->GetNumber("ty", 0.0) + 90 - LIMELIGHT_YTHETA;
    //   double height = vision.GetIDMapValue(2, id) - LIMELIGHT_HEIGHT;
    //   double distance = height/tan(Deg2Rad(theta)) - LIMELIGHT_DISPLACEMENT;
    //   DebugOutF("ty: " + std::to_string(vision.GetLimeLight()->GetNumber("ty", 0.0)));
    //   DebugOutF("theta: " + std::to_string(theta));
    //   DebugOutF("height: " + std::to_string(height));
    //   DebugOutF("distance: " + std::to_string(distance));
    // }
    // DebugOutF("shooter speed: " + std::to_string(GetArm().m_FlywheelPower));
  }));

  m_Print2.OnTrue(new frc2::InstantCommand([&] {
    // DebugOutF("button clicked");
    // Vision vision = GetVision();
    // if(vision.GetLimeLight()->GetNumber("tv", 0.0) == 1) {
    //   int id = vision.GetLimeLight()->GetNumber("tid", 0.0);
    //   double theta = vision.GetLimeLight()->GetNumber("ty", 0.0) + 90 - LIMELIGHT_YTHETA;
    //   double height = vision.GetIDMapValue(2, id) - LIMELIGHT_HEIGHT;
    //   double distance = height/tan(Deg2Rad(theta)) - LIMELIGHT_DISPLACEMENT;
    //   DebugOutF("ty: " + std::to_string(vision.GetLimeLight()->GetNumber("ty", 0.0)));
    //   DebugOutF("theta: " + std::to_string(theta));
    //   DebugOutF("height: " + std::to_string(height));
    //   DebugOutF("distance: " + std::to_string(distance));
    // }
  }));

  m_Print3.WhileTrue(new frc2::InstantCommand([&] {
    // DebugOutF("BL Voltage: " + std::to_string(GetDriveTrain().m_BackLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("BR Voltage: " + std::to_string(GetDriveTrain().m_BackRightModule.GetSteerSensorVoltage()));
    // DebugOutF("FL Voltage: " + std::to_string(GetDriveTrain().m_FrontLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("FR Voltage: " + std::to_string(GetDriveTrain().m_FrontRightModule.GetSteerSensorVoltage()));
    // DebugOutF("Max Sensor Voltage: " + std::to_string(frc::RobotController::GetVoltage5V()));
    DebugOutF("digital input 1: " + std::to_string(m_DustpanLaser.Get()));
    DebugOutF("digital input 2: " + std::to_string(m_UnderBotLaser.Get()));
  }));

  m_FlashLEDs.ToggleOnTrue(new frc2::InstantCommand([&] {
    m_LED.PickupFlashing();
  }));
}

// frc::Pose2d Robot::TransformPose(frc::Pose2d SelectedPose){ //rotating poses do not add correctly
// 	// if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 1){
// 	// 	SelectedPose = SelectedPose +
// 	// 		frc::Transform2d(
// 	// 			frc::Translation2d(units::meter_t(0), units::meter_t(1.68)),
// 	// 			frc::Rotation2d(units::radian_t(0))
// 	// 	).Inverse(); //delete inverse if not going 180
// 	// } else if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 2){
// 	// 	SelectedPose = SelectedPose + 
// 	// 		frc::Transform2d(
// 	// 			frc::Translation2d(units::meter_t(0), units::meter_t(2 * 1.68)),
// 	// 			frc::Rotation2d(units::radian_t(0))
// 	// 	).Inverse(); //delete inverse if not going 180	
// 	// }
// 	// if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
// 	// 	SelectedPose = 
// 	// 		frc::Pose2d(
// 	// 			units::meter_t(16.541)-SelectedPose.Translation().X(), 
// 	// 			SelectedPose.Translation().Y(),
// 	// 			SelectedPose.Rotation().RotateBy(Rotation2d(units::degree_t(180)))
// 	// 		);
// 	// }x
// 	return SelectedPose;
// }

pathplanner::PathPlannerTrajectory Robot::getTrajectory(std::string traj) {
  // Load the path you want to follow using its name in the GUI
  auto path = PathPlannerPath::fromPathFile(traj);
  // if(frc::DriverStation::GetAlliance() == DriverStation::Alliance::kRed) {
    // path = path->flipPath();
  // }
  startingPose = path.get()->getPreviewStartingHolonomicPose();
  return path.get()->getTrajectory(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s), path.get()->getPreviewStartingHolonomicPose().Rotation());
}

// /**
//  * This function is called every 20 ms, no matter the mode. Use
//  * this for items like diagnostics that you want to run during disabled,
//  * autonomous, teleoperated and test.
//  *
//  * <p> This runs after the mode specific periodic functions, but before
//  * LiveWindow and SmartDashboard integrated updating.
//  */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  GetCOB().GetTable().GetEntry("/COB/robotAngle").SetDouble(Robot::GetAngle());
  GetCOB().GetTable().GetEntry("/COB/matchTime").SetDouble(frc::DriverStation::GetMatchTime().value());
  GetCOB().GetTable().GetEntry("/COB/ticks").SetDouble(m_COBTicks);
  GetCOB().GetTable().GetEntry("/COB/odoX").SetDouble(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value());
  GetCOB().GetTable().GetEntry("/COB/odoY").SetDouble(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value());
  GetCOB().GetTable().GetEntry("/COB/odoZ").SetDouble(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value(), 360);

  GetCOB().GetTable().GetEntry("/COB/BackLeftDeviceTemp").SetString(std::to_string(GetDriveTrain().m_BackLeftModule.m_DriveController.motor.GetDeviceTemp().GetValue().value()));
  GetCOB().GetTable().GetEntry("/COB/BackRightDeviceTemp").SetString(std::to_string(GetDriveTrain().m_BackRightModule.m_DriveController.motor.GetDeviceTemp().GetValue().value()));
  GetCOB().GetTable().GetEntry("/COB/FrontLeftDeviceTemp").SetString(std::to_string(GetDriveTrain().m_FrontLeftModule.m_DriveController.motor.GetDeviceTemp().GetValue().value()));
  GetCOB().GetTable().GetEntry("/COB/FrontRightDeviceTemp").SetString(std::to_string(GetDriveTrain().m_FrontRightModule.m_DriveController.motor.GetDeviceTemp().GetValue().value()));

  GetCOB().GetTable().GetEntry("/COB/distanceToApr").SetDouble(GetVision().DistanceFromAprilTag(GetVision().GetLimeLight()->GetEntry("tid").GetInteger(0.0)));
  GetCOB().GetTable().GetEntry("/COB/flywheel%").SetDouble(GetArm().m_FlywheelPower);
  GetCOB().GetTable().GetEntry("/COB/stringpot").SetDouble(GetArm().GetStringPot().GetAverageValue());
  GetCOB().GetTable().GetEntry("/COB/angleAdjustment").SetDouble(m_AngleOffset);

  m_COBTicks++;
  GetRobot()->GetCOB().GetTable().GetEntry("/COB/pitchAngle").SetDouble(GetNavX().GetPitch() + 0.05);
  GetRobot()->GetCOB().GetTable().GetEntry("/COB/autoPath").SetString(m_AutoPath);
  
  // m_AutoPath = std::string(Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/auto").GetString(""));

  GetButtonBoard().SetOutputs(0);

  bool preset = false;
  double difference = 0;

  // LEDs!!!
  m_LED.SponsorBoardAllianceColor();
  m_LED.LaserSensors();
  m_LED.SetData();
  // m_LED.LowBattery();
  // m_LED.EyesAllianceColor();
  // m_LED.EndGame();
  // m_LED.SetData();
  // m_LED.SponsorBoardRainbow();
  // m_LED.LowBattery();                                                                                                                   
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }

  GetDriveTrain().BaseDrive(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));

  GetDriveTrain().BrakeMode(true);
  // GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // m_AutoFlag = true;
  DebugOutF("Auto init");

  frc2::CommandScheduler::GetInstance().CancelAll();
  GetNavX().ZeroYaw();
  GetNavX().Reset();
  // GetNavX().SetAngleAdjustment(m_AngleOffset);
  GetDriveTrain().BrakeMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  m_autonomousCommand = TrajectoryCommand(getTrajectory("testRot")).ToPtr();

  // does this whole thing make any difference??
  DebugOutF("before rotation: " + std::to_string(startingPose.Rotation().Degrees().value()));

  GetDriveTrain().GetOdometry()->ResetPosition(
    units::radian_t(Deg2Rad(GetAngle())), 
    GetDriveTrain().GetModulePositions(),
    startingPose
  );

  // if (m_autonomousCommand) {
  //   m_autonomousCommand->Schedule();
  // }

  
  frc2::CommandScheduler::GetInstance().Schedule(
    new frc2::SequentialCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(2.0_s),
        ConstantPivot(),
        //LockOn(),
        Flywheel(),
        frc2::SequentialCommandGroup(
          frc2::WaitCommand(1.0_s),
          Shoot()
        )
      ),
      frc2::InstantCommand([&] {
        GetArm().GetShooterMotor1().Set(0);
        GetArm().GetShooterMotor2().Set(0);
        GetArm().GetDustpanLaunchServo().Set(1);
      }),
      TrajectoryCommand(getTrajectory("testRot"))
    )
  );
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }

  GetDriveTrain().BaseDrive(frc::ChassisSpeeds(0_mps, 0_mps, 0_rad_per_s));

  DebugOutF("actual odometry position: \nx: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
  DebugOutF("y: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
  DebugOutF("rotation: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));

  // m_AutoFlag = false;
  // frc2::CommandScheduler::GetInstance().Schedule(new frc2::InstantCommand([&] {
  //     if(COB_GET_ENTRY(m_Vision.FrontBack("tv")).GetInteger(0) == 1 && COB_GET_ENTRY(m_Vision.FrontBack("botpose")).GetDoubleArray(std::span<double>()).size() != 0){
  //       frc::Pose2d startingPose = frc::Pose2d(m_Vision.GetFieldPose().Translation(), units::radian_t(Deg2Rad(GetAngle())));
  //       GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
  //       wpi::array<frc::SwerveModulePosition, 4>
  //             (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
  //       startingPose);
  //       DebugOutF("Pose Reset. X: " + std::to_string(startingPose.X().value()) + ", Y: " + std::to_string(startingPose.Y().value()) + ", Z: " + std::to_string(startingPose.Rotation().Degrees().value()));
  //     } else {
  //       DebugOutF("Pose Reset Fail");
  //     }
  //   })
  // );
  // m_MMT.MotionMagicTestInit();
  // m_LED.m_IsTele = true;  // used for LED Timer
  GetNavX().ZeroYaw();
  m_DriveTrain.BrakeMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

/**
 * This function is called periodically during operator control.  
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif