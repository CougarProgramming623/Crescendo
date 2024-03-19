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
#include <commands/LockOn.h>
#include <commands/AutoTest.h>


using ctre::phoenix::motorcontrol::ControlMode;
using namespace pathplanner;

Robot* Robot::s_Instance = nullptr;

Robot::Robot() :
m_NavX(frc::SerialPort::Port(2), AHRS::SerialDataType(0), uint8_t(66))
// m_LED()
{
  DebugOutF("inside robot constructor");
  s_Instance = this;
}


void Robot::RobotInit() {
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  s_Instance = this;
  DebugOutF("initalizing drivetrain w/ motors");
  m_DriveTrain.DriveInit();
  
  DebugOutF("initalizing motors finished");
  DebugOutF("x: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
  DebugOutF("y: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
  DebugOutF("theta: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
  // m_Vision.VisionInit(); //Make one
  m_Arm.Init();

  DebugOutF("BL Voltage: " + std::to_string(GetDriveTrain().m_BackLeftModule.GetSteerSensorVoltage()));
  DebugOutF("BR Voltage: " + std::to_string(GetDriveTrain().m_BackRightModule.GetSteerSensorVoltage()));
  DebugOutF("FL Voltage: " + std::to_string(GetDriveTrain().m_FrontLeftModule.GetSteerSensorVoltage()));
  DebugOutF("FR Voltage: " + std::to_string(GetDriveTrain().m_FrontRightModule.GetSteerSensorVoltage()));
  DebugOutF("Max Sensor Voltage: " + std::to_string(frc::RobotController::GetVoltage5V()));

  NamedCommands::registerCommand("AutoTest", std::move(AutoTest().ToPtr()));
  
  AutoButtons();
  // m_LED.Init();
  
  
  m_COBTicks = 0;
  m_AutoPath = "";
  m_ArmCommand = nullptr;
}

void Robot::AutoButtons() {
  //BUTTONBOARD
  // m_DustpanUpperLimit = m_ButtonBoard.GetRawAxis(DUSTPANUP_LIMIT);//(BUTTON_L(DUSTPANUP_LIMIT))
  // frc::AnalogInput m_ShooterSpeed;
  
  // frc2::Trigger m_IntakeSwitch;
  // frc2::Trigger m_FlywheelSwitch;
  // m_ArmOverride = frc2::Trigger(BUTTON_L(ARM_OVERRIDE));
	// m_ShooterUp = frc2::Trigger(BUTTON_L(SHOOTER_UP));
	// m_ShooterDown = frc2::Trigger(BUTTON_L(SHOOTER_DOWN));
  // m_VisionAim = frc2::Trigger(BUTTON_L(AIM_BUTTON));
  // m_Shoot = frc2::Trigger(BUTTON_L(SHOOT_BUTTON));
  // m_AmpPreset = frc2::Trigger(BUTTON_L(AMP_BUTTON));
  // m_StowPreset = frc2::Trigger(BUTTON_L(STOW_BUTTON));

  // m_VisionAim.OnTrue(new frc2::InstantCommand([&] {
	// 	frc2::PrintCommand("VISION AIM");
	// }));

  // frc2::Trigger m_AllUp;
  // frc2::Trigger m_ClimbUp;
  // frc2::Trigger m_ClimbDown;
  // frc2::Trigger m_DustpanDown;
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
// 	// }
// 	return SelectedPose;
// }

frc2::CommandPtr Robot::getAutonomousCommand() {
  // Load the path you want to follow using its name in the GUI
  auto path = PathPlannerPath::fromPathFile("StraightX");
  DebugOutF(std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
  DebugOutF(std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
  DebugOutF(std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
  // Create a path following command using AutoBuilder. This will also trigger event markers.
  startingPose = Pose2d(path.get()->getAllPathPoints().at(0).position, path.get()->getAllPathPoints().at(0).rotationTarget.value().getTarget());
  DebugOutF(std::to_string(startingPose.X().value()));
  DebugOutF(std::to_string(startingPose.Y().value()));
  //startingPose = path.get()->getStartingDifferentialPose();
  return AutoBuilder::followPathWithEvents(path);
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
  Robot::GetCOB().GetTable().GetEntry("/COB/robotAngle").SetDouble(Robot::GetAngle());
  Robot::GetCOB().GetTable().GetEntry("/COB/matchTime").SetDouble(frc::DriverStation::GetMatchTime().value());
  Robot::GetCOB().GetTable().GetEntry("/COB/ticks").SetDouble(m_COBTicks);
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaX").SetDouble(std::abs(GetDriveTrain().m_VisionRelative.X().value()));
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaY").SetDouble(std::abs(GetDriveTrain().m_VisionRelative.Y().value()));
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaT").SetDouble(std::abs(-fmod(360 - GetDriveTrain().m_VisionRelative.Rotation().Degrees().value(), 360)));
  // m_Vision.PushID();

  m_COBTicks++;
  Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/pitchAngle").SetDouble(Robot::GetRobot()->GetNavX().GetPitch() + 0.05);
  // m_AutoPath = std::string(Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/auto").GetString(""));
  m_AutoPath = "New New Path";

  if(Robot::GetButtonBoard().GetRawButton(15)){
    DebugOutF("FL: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontLeftModule.GetSteerAngle())));
    DebugOutF("FR: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontRightModule.GetSteerAngle())));
    DebugOutF("BL: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackLeftModule.GetSteerAngle())));
    DebugOutF("BR: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackRightModule.GetSteerAngle())));
  }

  if(GetButtonBoard().GetRawButton(16)) {
    DebugOutF("Stringpot Value: " + std::to_string(GetArm().GetStringPot().GetValue()));
    // DebugOutF("BL Voltage: " + std::to_string(GetDriveTrain().m_BackLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("BR Voltage: " + std::to_string(GetDriveTrain().m_BackRightModule.GetSteerSensorVoltage()));
    // DebugOutF("FL Voltage: " + std::to_string(GetDriveTrain().m_FrontLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("FR Voltage: " + std::to_string(GetDriveTrain().m_FrontRightModule.GetSteerSensorVoltage()));
    // DebugOutF("Max Sensor Voltage: " + std::to_string(frc::RobotController::GetVoltage5V()));
  }

  bool preset = false;
  double difference = 0;

  GetButtonBoard().SetOutputs(0xffffffff);

  if (GetButtonBoard().GetRawButton(19)) {
    GetArm().GetClimbMotor().SetControl(m_DutyCycleOutRequest.WithOutput(0.5));
  } else if (GetButtonBoard().GetRawButton(20)) {
    GetArm().GetClimbMotor().SetControl(m_DutyCycleOutRequest.WithOutput(-0.5));
  } else if (!GetButtonBoard().GetRawButton(19) && !GetButtonBoard().GetRawButton(20)) {
    GetArm().GetClimbMotor().SetControl(m_DutyCycleOutRequest.WithOutput(0));
  }

  if(GetButtonBoard().GetRawButton(7)) {
    preset = true;
    // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
    // GetArm().GetPivotMotor().SetControl(m_DutyCycleOutRequest.WithOutput(GetArm().m_OriginalPivotRotations - GetArm().PivotStringPotUnitsToRotations(GetArm().m_StringPotOffset)));
    // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
    // DebugOutF("stringpot value: " + std::to_string(GetArm().GetStringPot().GetValue()));
    // DebugOutF("preset value: " + CLOSEUPSHOOTSTRINGPOT);
    difference = GetRobot()->GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
    DebugOutF("difference: " + std::to_string(difference));
    if(GetRobot()->GetArm().GetStringPot().GetValue() != CLOSEUPSHOOTSTRINGPOT) {
      if(difference > 0) {
        GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3));
      } else if(difference < 0) {
        GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
      }
    }
    preset = false;
  }

  if(GetButtonBoard().GetRawButton(8)) {
    preset = true;
    // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
    // GetArm().GetPivotMotor().SetControl(m_DutyCycleOutRequest.WithOutput(GetArm().m_OriginalPivotRotations - GetArm().PivotStringPotUnitsToRotations(GetArm().m_StringPotOffset)));
    // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - PICKUPSTRINGPOT;
    if(GetRobot()->GetArm().GetStringPot().GetValue() != PICKUPSTRINGPOT) {
      difference = GetRobot()->GetArm().GetStringPot().GetValue() - PICKUPSTRINGPOT;
      if(difference > 0) {
        GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3));
      } else if(difference < 0) {
        GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
      }
    }
    preset = false;
  }

  if(GetButtonBoard().GetRawButton(18)) {
    GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3));
  } else if(GetButtonBoard().GetRawButton(17) && GetArm().GetStringPot().GetValue() < 815) {
    GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
  } else if(!GetButtonBoard().GetRawButton(17) && !GetButtonBoard().GetRawButton(18) && !preset) {
    GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  }

  if(GetButtonBoard().GetRawButton(21)){
    GetRobot()->GetDriveTrain().m_DustpanRotate.Set(0);
  }
  
  if(GetButtonBoard().GetRawButton(22)) {
    GetRobot()->GetDriveTrain().m_DustpanRotate.Set(1);
  }

  if(GetButtonBoard().GetRawButton(5)){
    GetRobot()->GetDriveTrain().m_DustpanLaunch.Set(0.75);
  } else if(!GetButtonBoard().GetRawButton(5)) {
    GetRobot()->GetDriveTrain().m_DustpanLaunch.Set(1);
  }

  if(GetButtonBoard().GetRawButton(1)) {
    double output = GetButtonBoard().GetRawAxis(SHOOTER_SPEED);
    GetArm().GetShooterMotor1().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(output - 0.05));
    GetArm().GetShooterMotor2().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(output));
  } else if(!GetButtonBoard().GetRawButton(1)) {
    GetArm().GetShooterMotor1().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    GetArm().GetShooterMotor2().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  }

  if(GetButtonBoard().GetRawButton(2)) {
    GetArm().GetFeeder().Set(ControlMode::PercentOutput, 0.25);
  } else if(!GetButtonBoard().GetRawButton(2)) {
    GetArm().GetFeeder().Set(ControlMode::PercentOutput, 0);
  }

  //LED functionality
  // m_LED.SponsorBoardAllianceColor();
  // m_LED.LowBattery();
  // m_LED.EyesAllianceColor();
  // m_LED.EndGame();
  // m_LED.SetData();
  // m_LED.SponsorBoardRainbow();
  // m_LED.LowBattery();

  // Robot::GetCOB().GetTable().GetEntry("/COB/armValue").SetDouble(Robot::GetArm().GetPot());   
  // Robot::GetCOB().GetTable().GetEntry("/COB/armAngle").SetDouble(Robot::GetArm().PivotTicksToDeg(Robot::GetArm().GetPivot().GetSelectedSensorPosition()));                                                                                                                                   
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
  m_AutoFlag = true;
  DebugOutF("Auto init");

  frc2::CommandScheduler::GetInstance().CancelAll();
  GetNavX().ZeroYaw();
  GetNavX().Reset();
  GetNavX().SetAngleAdjustment(0);
  GetDriveTrain().BrakeMode(true);
  // GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  GetDriveTrain().GetOdometry()->ResetPosition(
    units::radian_t(Deg2Rad(GetAngle())), 
    wpi::array<frc::SwerveModulePosition, 4>(
      GetDriveTrain().m_FrontLeftModule.GetPosition(), 
      GetDriveTrain().m_FrontRightModule.GetPosition(), 
      GetDriveTrain().m_BackLeftModule.GetPosition(), 
      GetDriveTrain().m_BackRightModule.GetPosition()),
    startingPose
    );

  m_autonomousCommand = getAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }

  /*m_AutoPath = "Test";
  m_autonomousCommand = getAutonomousCommand();
  frc2::CommandScheduler::GetInstance().Schedule(
    new frc2::SequentialCommandGroup(
      frc2::ParallelCommandGroup(
        frc2::ParallelCommandGroup(
          frc2::WaitCommand(1.5_s),
          frc2::InstantCommand([&] {
            GetRobot()->GetDriveTrain().m_DustpanLaunch.Set(0.75);
          })
        ),
        frc2::InstantCommand([] {
          DebugOutF("where shooter motors were");
          // Robot::GetRobot()->GetArm().GetShooterMotor1().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3 + 0.05));
          // Robot::GetRobot()->GetArm().GetShooterMotor2().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3));
        })
      ),
      frc2::InstantCommand([] {
        Robot::GetRobot()->m_autonomousCommand;
      })
    )
  );*/
}
void Robot::AutonomousPeriodic() {
    // DebugOutF("X: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
    // DebugOutF("Y: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
    // DebugOutF("Deg: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
    DebugOutF("(x, y): " + std::to_string(GetDriveTrain().getPose().X().value()) + ", " + std::to_string(GetDriveTrain().getPose().Y().value()));
}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }

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
  // GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  // GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  GetNavX().SetAngleAdjustment(0);
   
  // frc::Pose2d startingPose = frc::Pose2d(units::meter_t(2.54), units::meter_t(1.75), frc::Rotation2d(units::degree_t(0)));
  //   GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
  //       wpi::array<frc::SwerveModulePosition, 4>
  //           (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
  //       startingPose);

  // m_Arm.PlaceElement(0,0);
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
