// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include "Util.h"
//#include "commands/TrajectoryCommand.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/RobotController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandPtr.h>
#include "commands/LockOn.h"
#include <frc/DriverStation.h>


//using ctre::phoenix::motorcontrol::ControlMode;
using namespace pathplanner;

Robot* Robot::s_Instance = nullptr;

Robot::Robot() :
m_NavX(frc::SerialPort::Port(2), AHRS::SerialDataType(0), uint8_t(66)),
m_Intake()
{
  s_Instance = this;
}

void Robot::RobotInit() {

  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  s_Instance = this;
  m_DriveTrain.DriveInit();
  m_Vision.VisionInit(); //Make one
  // m_LED.Init();
  m_Arm.Init();
  
  AutoButtons();
  
  m_COBTicks = 0;
  m_AutoPath = "";
  m_ArmCommand = nullptr;
  m_AutoFlag = true;

  if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    m_ColOffset = 2;
  } else {
    m_ColOffset = 0;
  }
}

void Robot::AutoButtons(){
  //BUTTONBOARD 2
  m_BigRed      = frc2::Trigger(BUTTON_L(BIG_RED));

  m_SingleSub   = frc2::Trigger(BUTTON_L(5));
  m_DoubleSub = frc2::Trigger(BUTTON_L_TWO(13));
  m_SingleSubCube = frc2::Trigger(BUTTON_L(7));

  m_LeftGrid    = frc2::Trigger(BUTTON_L_TWO(LEFT_GRID));
  m_CenterGrid  = frc2::Trigger(BUTTON_L_TWO(CENTER_GRID));
  m_RightGrid   = frc2::Trigger(BUTTON_L_TWO(RIGHT_GRID));

  m_PlacingMode = frc2::Trigger(BUTTON_L_TWO(PLACING_MODE));
  m_GroundPickup = frc2::Trigger(BUTTON_L_TWO(GROUND_PICKUP_MODE));

  m_NavXReset = frc2::Trigger(BUTTON_L(8)); //PUT Define
  GetArm().m_PlacingMode = frc2::Trigger(BUTTON_L_TWO(15));
  m_AutoBalance = frc2::Trigger(BUTTON_L(3));
  m_Print = frc2::Trigger(BUTTON_L(2));
  m_SysId = frc2::Trigger(BUTTON_L(1));

  m_VisionPoseReset = frc2::Trigger([&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(6); }); //PUT Define


  m_NavXReset.OnTrue(
    new frc2::InstantCommand([&]{
      DebugOutF("NavX Zero");
      zeroGyroscope();
  }));

  m_VisionPoseReset.OnTrue(
    new frc2::InstantCommand([&] {
      DebugOutF("Pose Resetting");
      if(COB_GET_ENTRY(m_Vision.FrontBack("tv")).GetInteger(0) == 1 && COB_GET_ENTRY(m_Vision.FrontBack("botpose")).GetDoubleArray(std::span<double>()).size() != 0){
        frc::Pose2d startingPose = frc::Pose2d(m_Vision.GetPoseBlue().Translation(), units::radian_t(Deg2Rad(GetAngle())));
        GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
        wpi::array<frc::SwerveModulePosition, 4>
              (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
        startingPose);
        DebugOutF("Pose Reset. X: " + std::to_string(startingPose.X().value()) + ", Y: " + std::to_string(startingPose.Y().value()) + ", Z: " + std::to_string(startingPose.Rotation().Degrees().value()));
      } else {
        DebugOutF("Pose Reset Fail");
      }
    })
  );



  (frc2::Trigger(BUTTON_L(8)) && Robot::GetRobot()->m_SysId).WhileTrue(
      Robot::GetRobot()->GetDriveTrain().SysIdQuasistatic(frc2::sysid::Direction::kForward)
      // new frc2::InstantCommand([&] { GetDriveTrain().m_FrontRightModule.m_DriveController.GetMotor().SetPosition(units::angle::turn_t(0)); })
  );

  (frc2::Trigger(BUTTON_L(6)) && Robot::GetRobot()->m_SysId).WhileTrue(
      Robot::GetRobot()->GetDriveTrain().SysIdQuasistatic(frc2::sysid::Direction::kReverse)
      // new frc2::InstantCommand([&] { GetDriveTrain().m_FrontRightModule.m_DriveController.GetMotor().SetPosition(units::angle::turn_t(1500)); })
  );

  (frc2::Trigger(BUTTON_L(7)) && Robot::GetRobot()->m_SysId).WhileTrue(
      Robot::GetRobot()->GetDriveTrain().SysIdDynamic(frc2::sysid::Direction::kForward)
      // new frc2::InstantCommand([&] { GetDriveTrain().m_FrontRightModule.m_DriveController.GetMotor().SetPosition(units::angle::turn_t(500)); })
  );

  (frc2::Trigger(BUTTON_L(5)) && Robot::GetRobot()->m_SysId).WhileTrue(
      Robot::GetRobot()->GetDriveTrain().SysIdDynamic(frc2::sysid::Direction::kReverse)
      // new frc2::InstantCommand([&] { GetDriveTrain().m_FrontRightModule.m_DriveController.GetMotor().SetPosition(units::angle::turn_t(1000)); })
  );
}



frc::Pose2d Robot::TransformPose(frc::Pose2d SelectedPose){ //rotating poses do not add correctly
	if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 1){
		SelectedPose = SelectedPose +
			frc::Transform2d(
				frc::Translation2d(units::meter_t(0), units::meter_t(1.68)),
				frc::Rotation2d(units::radian_t(0))
		).Inverse(); //delete inverse if not going 180
	} else if(Robot::GetRobot()->GetDriveTrain().m_SelectedGrid == 2){
		SelectedPose = SelectedPose + 
			frc::Transform2d(
				frc::Translation2d(units::meter_t(0), units::meter_t(2 * 1.68)),
				frc::Rotation2d(units::radian_t(0))
		).Inverse(); //delete inverse if not going 180	
	}
	if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
		SelectedPose = 
			frc::Pose2d(
				units::meter_t(16.541)-SelectedPose.Translation().X(), 
				SelectedPose.Translation().Y(),
				SelectedPose.Rotation().RotateBy(Rotation2d(units::degree_t(180)))
			);
	}
	return SelectedPose;
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  Robot::GetCOB().GetTable().GetEntry("/COB/robotAngle").SetDouble(Robot::GetAngle());   
  Robot::GetCOB().GetTable().GetEntry("/COB/matchTime").SetDouble(frc::DriverStation::GetMatchTime().value());
  Robot::GetCOB().GetTable().GetEntry("/COB/ticks").SetDouble(m_COBTicks);
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaX").SetDouble(std::abs(GetDriveTrain().m_VisionRelative.X().value()));
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaY").SetDouble(std::abs(GetDriveTrain().m_VisionRelative.Y().value()));
  Robot::GetCOB().GetTable().GetEntry("/COB/deltaT").SetDouble(std::abs(-fmod(360 - GetDriveTrain().m_VisionRelative.Rotation().Degrees().value(), 360)));
  m_Vision.PushID();

  m_COBTicks++;
  Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/pitchAngle").SetDouble(Robot::GetRobot()->GetNavX().GetPitch() + 0.05);
  m_AutoPath = std::string(Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/auto").GetString(""));
  // DebugOutF("Row: " + std::to_string(SelectedRow) + " , Col: " + std::to_string(SelectedColumn));

  if(Robot::GetButtonBoard().GetRawButton(4)){
    // DebugOutF("BL: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackLeftModule.GetSteerAngle())));
    // DebugOutF("BR: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackRightModule.GetSteerAngle())));
    // DebugOutF("FL: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontLeftModule.GetSteerAngle())));
    // DebugOutF("FR: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontRightModule.GetSteerAngle())));
    DebugOutF("front right drive position: " + std::to_string(GetDriveTrain().m_FrontRightModule.m_DriveController.GetMotor().GetPosition().GetValueAsDouble()));
  }

  // if(m_SysId.Get()) {
  //   Robot::GetRobot()->GetDriveTrain().SysIdQuasistatic(frc2::sysid::Direction::kForward);
  // }

  // m_Routine.ConfigureBindings();

  //LED
  // m_LED.SponsorBoardAllianceColor();
  // m_LED.LowBattery();
  // m_LED.EyesAllianceColor();
  // m_LED.EndGame();
  // m_LED.SetData();
  //m_LED.SponsorBoardRainbow();
  //m_LED.LowBattery();


  // DebugOutF("PosDeg: " + std::to_string(GetArm().WristTicksToDegrees(GetArm().GetWristMotor().GetSelectedSensorPosition())));
	// DebugOutF("PosTicks: " + std::to_string(GetArm().GetWristMotor().GetSelectedSensorPosition()));

  // DebugOutF(std::to_string(m_Arm.m_WristPos));

  // DebugOutF("CanCoder" + std::to_string(Robot::GetRobot()->GetArm().GetPivotCANCoder().GetAbsolutePosition()));
  // DebugOutF("Arm" + std::to_string(Robot::GetRobot()->GetArm().PivotTicksToDegrees(Robot::GetRobot()->GetArm().GetPivotMotor().GetSelectedSensorPosition())));

  // DebugOutF("X: " + std::to_string(Robot::GetRobot()->GetDriveTrain().m_TransformedPose.X().value()));
  // DebugOutF("Y: " + std::to_string(Robot::GetRobot()->GetDriveTrain().m_TransformedPose.Y().value()));
  // DebugOutF("Deg: " + std::to_string(Robot::GetRobot()->GetDriveTrain().m_TransformedPose.Rotation().Degrees().value()));

  // Robot::GetCOB().GetTable().GetEntry("/COB/armValue").SetDouble(Robot::GetArm().GetPot());   
  // Robot::GetCOB().GetTable().GetEntry("/COB/armAngle").SetDouble(Robot::GetArm().PivotTicksToDeg(Robot::GetArm().GetPivot().GetSelectedSensorPosition()));                                                                                                                                   
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  // GetDriveTrain().BreakMode(true);
  // GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  
  m_AutoFlag = true;
  DebugOutF("AutonomousInit");

  // frc2::CommandScheduler::GetInstance().CancelAll();
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  // GetDriveTrain().BreakMode(true);
  // GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);


  // SysIdCommand().Schedule();
  // frc2::CommandScheduler().Run();

  // m_AutoCommand = m_Routine.GetAutonomousCommand();

  // if(m_AutoCommand) {
  //   m_AutoCommand->Schedule();
  // }

  //PathPlannerTrajectory traj;

  //Load trajectory
  // if(!COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
    // DebugOutF("Blue");
  //PathPlannerTrajectory traj = PathPlanner::loadPath(m_AutoPath, PathConstraints(4_mps, 1_mps_sq));

  // PathPlannerPath traj = PathPlannerPath::fromPathFile("AutoBalanceExtra", PathConstraints(4_mps, 1.5_mps_sq, 4_rad_per_s, 1.5_rad_per_s_sq));

  // // } else {
  // // //   DebugOutF("Red");
  // // //   PathPlannerTrajectory traj = PathPlanner::loadPath("TestBalanceRed", PathConstraints(4_mps, 1_mps_sq));
  // // // }

  // //PathPlannerTrajectory::transformTrajectoryForAlliance(traj, frc::DriverStation::GetAlliance());

  // frc::Pose2d startingPose = frc::Pose2d(traj.getInitialState().pose.Translation(), frc::Rotation2d(units::degree_t(0)));

  // GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
  //   wpi::array<frc::SwerveModulePosition, 4>
  //        (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
  //   startingPose);
  
  
  // DebugOutF("InitialRotation: " + std::to_string(traj.getInitialHolonomicPose().Rotation().Degrees().value()));
  // DebugOutF("InitialY: " + std::to_string(traj.asWPILibTrajectory().InitialPose().Y().value()));
  // DebugOutF("InitialX: " + std::to_string(traj.asWPILibTrajectory().InitialPose().X().value()));
  
  //DEFINITELY LOOK - COMMENTING OUT AUTO FOR NOW TO FOCUS ON REIMPLEMENTING OTHER FUNCTIONALITY
  if(true/*COB_GET_ENTRY("/COB/autos").GetString("") == "Auto1"*/){
  //   frc2::CommandScheduler::GetInstance().Schedule(
  //     new frc2::SequentialCommandGroup(
  //     frc2::ParallelRaceGroup(
  //       frc2::WaitCommand(1.8_s),
  //       PivotToPosAuto(-22.0), 
  //       frc2::FunctionalCommand(
  //         [&] {
  //           GetArm().GetBottomIntakeMotor().EnableCurrentLimit(false);
  //           GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -1);
  //         },
  //         [&] {},
  //         [&](bool e) { // onEnd
  //           GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
  //         },
  //         [&] { // isFinished
  //         return false;
  //         }
  //       ),
  //       WristToPosAuto(28.0)
  //     ),

  //     frc2::ParallelRaceGroup(
  //       frc2::FunctionalCommand(
  //         [&] {
  //           GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 1);
  //         },
  //         [&] {},
  //         [&](bool e) { // onEnd
  //           GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
  //           GetArm().GetBottomIntakeMotor().EnableCurrentLimit(true);
  //         },
  //         [&] { // isFinished
  //         return false;
  //         }
  //       ),
  //       frc2::WaitCommand(0.8_s)
  //     ),

  //   frc2::ParallelDeadlineGroup(
  //       //TrajectoryCommand(traj),
  //     PivotToPosAuto(92.0), 
  //     WristToPosAuto(120)
  //   ),
  //   AutoBalance()
    
  // ));
  //DebugOutF(GetDriveTrain().m_EventMap.find("\"Mark 1\""));
  // (GetDriveTrain().m_EventMap.at(std::string("Mark 1")).get()->Schedule());
  // } else if (COB_GET_ENTRY("/COB/autos").GetString("") == "Auto2"){
  //   frc2::CommandScheduler::GetInstance().Schedule(
  //     new frc2::SequentialCommandGroup(

  //       frc2::ParallelRaceGroup(
  //         frc2::WaitCommand(1.8_s),
  //         PivotToPosAuto(-22.0), 
  //         frc2::FunctionalCommand(
  //           [&] {
  //             GetArm().GetBottomIntakeMotor().EnableCurrentLimit(false);
  //             GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -1);
  //           },
  //           [&] {},
  //           [&](bool e) { // onEnd
  //             GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
  //           },
  //           [&] { // isFinished
  //           return false;
  //           }
  //         ),
  //         WristToPosAuto(28.0)
  //       ),

  //       frc2::ParallelRaceGroup(
  //         frc2::FunctionalCommand(
  //           [&] {
  //             GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 1);
  //           },
  //           [&] {},
  //           [&](bool e) { // onEnd
  //             GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
  //             GetArm().GetBottomIntakeMotor().EnableCurrentLimit(true);
  //           },
  //           [&] { // isFinished
  //           return false;
  //           }
  //         ),
  //         frc2::WaitCommand(0.8_s)
  //       ),

  //       frc2::ParallelDeadlineGroup(
  //         TrajectoryCommand(traj),
  //         frc2::SequentialCommandGroup(
  //           frc2::ParallelRaceGroup(
  //             frc2::WaitCommand(3.5_s), //FIX:: not sure abt time
  //             PivotToPosAuto(92.0), 
  //             WristToPosAuto(120)
  //           ),
  //           frc2::ParallelRaceGroup(
  //             frc2::WaitCommand(1.5_s), //FIX: not sure abt time
  //             PivotToPosAuto(94.0),
  //             WristToPosAuto(-2.0),
  //             frc2::FunctionalCommand(
  //               [&] {
  //               GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, -1);
  //               },
  //               [&] {},
  //               [&](bool e) { // onEnd
  //                 GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
  //                 GetArm().GetBottomIntakeMotor().EnableCurrentLimit(true);
  //               },
  //             [&] { // isFinished
  //               return false;
  //             }
  //             )
  //           ),
  //           frc2::ParallelRaceGroup(
  //             frc2::WaitCommand(1.0_s),
  //             PivotToPosAuto(92.0), 
  //             WristToPosAuto(127)
  //           )
  //         )
  //       ),
  //       AutoBalance()
  //   ));
  // //DebugOutF(GetDriveTrain().m_EventMap.find("\"Mark 1\""));
  // // (GetDriveTrain().m_EventMap.at(std::string("Mark 1")).get()->Schedule());
  // }
  }
}
void Robot::AutonomousPeriodic() {

    // DebugOutF("X: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
    // DebugOutF("Y: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
    // DebugOutF("Deg: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
  
}

void Robot::TeleopInit() {
  if(m_AutoCommand) {
    m_AutoCommand->Cancel();
  }

  m_AutoFlag = false;
  frc2::CommandScheduler::GetInstance().Schedule(new frc2::InstantCommand([&] {
      if(COB_GET_ENTRY(m_Vision.FrontBack("tv")).GetInteger(0) == 1 && COB_GET_ENTRY(m_Vision.FrontBack("botpose")).GetDoubleArray(std::span<double>()).size() != 0){
        frc::Pose2d startingPose = frc::Pose2d(m_Vision.GetPoseBlue().Translation(), units::radian_t(Deg2Rad(GetAngle())));
        GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
        wpi::array<frc::SwerveModulePosition, 4>
              (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
        startingPose);
        DebugOutF("Pose Reset. X: " + std::to_string(startingPose.X().value()) + ", Y: " + std::to_string(startingPose.Y().value()) + ", Z: " + std::to_string(startingPose.Rotation().Degrees().value()));
      } else {
        DebugOutF("Pose Reset Fail");
      }
    })
  );
  // m_MMT.MotionMagicTestInit();
  // m_LED.m_IsTele = true;  // used for LED Timer
  //GetNavX().ZeroYaw();
  // m_DriveTrain.BreakMode(true);
  // GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  // GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
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
void Robot::TeleopPeriodic() {
  //DebugOutF("Theta: " + std::to_string(GetAngle())); 
  
  // DebugOutF("LLX: " + std::to_string(m_Vision.GetPoseBlue().X().value()));
  // DebugOutF("LLY: " + std::to_string(m_Vision.GetPoseBlue().Y().value()));
  // DebugOutF("LLZ: " + std::to_string(m_Vision.GetPoseBlue().Rotation().Degrees().value()));
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
