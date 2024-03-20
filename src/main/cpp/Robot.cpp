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

//#include <commands/DriveToPosCommand.h>
//#include <commands/DualMotorControl.h>
//#include <commands/PivotToPosAuto.h>
//#include <commands/TrajectoryCommand.h>
//#include <commands/WristToPosAuto.h>


using ctre::phoenix::motorcontrol::ControlMode;
using namespace pathplanner;

Robot* Robot::s_Instance = nullptr;

Robot::Robot() :
m_NavX(frc::SerialPort::Port(1), AHRS::SerialDataType(0), uint8_t(66))
// m_LED()
{
  DebugOutF("inside robot constructor");
  s_Instance = this;

  NamedCommands::registerCommand("AutoTest", std::move(AutoTest().ToPtr()));

  // NamedCommands::registerCommand("lockOn", std::move(LockOn().ToPtr()));
  // //NamedCommands::registerCommand("driveToPosCommand", std::move(DriveToPosCommand().ToPtr())); 
  // NamedCommands::registerCommand("driveWithJoystick", std::move(DriveWithJoystick().ToPtr()));
  // //NamedCommands::registerCommand("dualMotorControl", std::move(DualMotorControl().ToPtr()));
  // NamedCommands::registerCommand("pivotToPos", std::move(PivotToPos().ToPtr())); 
  //NamedCommands::registerCommand("pivotToPosAuto", std::move(PivotToPosAuto().ToPtr()));
  //NamedCommands::registerCommand("trajectoryCommand", std::move(TrajectoryCommand().ToPtr()));
  // NamedCommands::registerCommand("wristToPos", std::move(WristToPos().ToPtr())); 
  //NamedCommands::registerCommand("wristToPosAuto", std::move(WristToPosAuto().ToPtr()));
}


void Robot::RobotInit() {
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  s_Instance = this;
  DebugOutF("initalizing drivetrain w/ motors");
  m_DriveTrain.DriveInit();

  // GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetInverted(false);
  // GetDriveTrain().m_FrontLeftModule.m_DriveController.motor.SetInverted(false);
  // GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetInverted(false);
  // GetDriveTrain().m_FrontRightModule.m_DriveController.motor.SetInverted(false);
  // GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetInverted(false);
  // GetDriveTrain().m_BackLeftModule.m_DriveController.motor.SetInverted(false);
  // GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetInverted(false);
  // GetDriveTrain().m_BackRightModule.m_DriveController.motor.SetInverted(false);
  
  DebugOutF("initalizing motors finished");
  DebugOutF("x: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
  DebugOutF("y: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
  DebugOutF("theta: " + std::to_string(Robot::GetRobot()->GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
  // m_Vision.VisionInit(); //Make one
  // m_LED.Init();
  // m_Arm.Init();

  DebugOutF("BL Voltage: " + std::to_string(GetDriveTrain().m_BackLeftModule.GetSteerSensorVoltage()));
  DebugOutF("BR Voltage: " + std::to_string(GetDriveTrain().m_BackRightModule.GetSteerSensorVoltage()));
  DebugOutF("FL Voltage: " + std::to_string(GetDriveTrain().m_FrontLeftModule.GetSteerSensorVoltage()));
  DebugOutF("FR Voltage: " + std::to_string(GetDriveTrain().m_FrontRightModule.GetSteerSensorVoltage()));
  DebugOutF("Max Sensor Voltage: " + std::to_string(frc::RobotController::GetVoltage5V()));
  
  AutoButtons();
  // m_LED.Init();
  
  
  m_COBTicks = 0;
  m_AutoPath = "";
  m_ArmCommand = nullptr;

  // if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
  //   m_ColOffset = 2;
  // } else {
  //   m_ColOffset = 0;
  // }
}

void Robot::AutoButtons() {
  //BUTTONBOARD
  // m_DustpanUpperLimit = m_ButtonBoard.GetRawAxis(DUSTPANUP_LIMIT);//(BUTTON_L(DUSTPANUP_LIMIT))
  // frc::AnalogInput m_ShooterSpeed;
  
  // frc2::Trigger m_FlywheelSwitch;
  // m_ArmOverride = frc2::Trigger(BUTTON_L(ARM_OVERRIDE));
	// m_ShooterUp = frc2::Trigger(BUTTON_L(SHOOTER_UP));
	// m_ShooterDown = frc2::Trigger(BUTTON_L(SHOOTER_DOWN));
  // m_VisionAim = frc2::Trigger(BUTTON_L(AIM_BUTTON));
  // m_Shoot = frc2::Trigger(BUTTON_L(SHOOT_BUTTON));
  	m_ArmOverride =  frc2::Trigger(BUTTON_L(ARM_OVERRIDE));
    m_ShooterUp = frc2::Trigger(BUTTON_L(SHOOTER_UP));
    m_ShooterDown = frc2::Trigger(BUTTON_L(SHOOTER_DOWN));
    m_RunFlywheel = frc2::Trigger(BUTTON_L(FLYWHEEL_SWITCH));
    m_FlywheelPowerLock = frc2::Trigger(BUTTON_L(SHOOTER_LOCK_POWER));
    m_DustpanUp = frc2::Trigger(BUTTON_L(DUSTPAN_UP));
    m_DustpanDown = frc2::Trigger(BUTTON_L(DUSTPAN_DOWN));
    m_ClimbUp = frc2::Trigger(BUTTON_L(CLIMB_UP));
    m_ClimbDown = frc2::Trigger(BUTTON_L(CLIMB_DOWN));
    m_IntakeSwitch = frc2::Trigger(BUTTON_L(INTAKE_SWITCH));
  //m_AmpPreset = frc2::Trigger(BUTTON_L(AMP_BUTTON));
  //m_StowPreset = frc2::Trigger(BUTTON_L(STOW_BUTTON));

  Robot::GetRobot()->m_FlywheelPowerLock.OnTrue(new frc2::InstantCommand([&] {
		GetArm().m_FlywheelPower = Robot::GetRobot()->GetButtonBoard().GetRawAxis(SHOOTER_SPEED);
	}));
	m_ArmOverride.OnTrue(GetArm().ManualControls());
	m_RunFlywheel.OnTrue(new Flywheel());
	m_IntakeSwitch.OnTrue(new Intake());

  // m_VisionAim.OnTrue(new frc2::InstantCommand([&] {
	// 	frc2::PrintCommand("VISION AIM");
	// }));

  // frc2::Trigger m_AllUp;
  // frc2::Trigger m_ClimbUp;
  // frc2::Trigger m_ClimbDown;
  // frc2::Trigger m_DustpanDown;

  //BUTTONBOARD 2
  // m_TL = frc2::Trigger(BUTTON_L_TWO(GRID_TL));
  // m_TL          = frc2::JoystickButton(BUTTON_L_TWO(GRID_TL));
  // m_TC          = frc2::JoystickButton(BUTTON_L_TWO(GRID_TC));
  // m_TR          = frc2::JoystickButton(BUTTON_L_TWO(GRID_TR));
  // m_ML          = frc2::JoystickButton(BUTTON_L_TWO(GRID_ML));
  // m_MC          = frc2::JoystickButton(BUTTON_L_TWO(GRID_MC));
  // m_MR          = frc2::JoystickButton(BUTTON_L_TWO(GRID_MR));
  // m_BL          = frc2::JoystickButton(BUTTON_L_TWO(GRID_BL));
  // m_BC          = frc2::JoystickButton(BUTTON_L_TWO(GRID_BC));
  // m_BR          = frc2::JoystickButton(BUTTON_L_TWO(GRID_BR));
  //m_BigRed      = frc2::Trigger(BUTTON_L(BIG_RED));

  // m_SingleSub   = frc2::Trigger(BUTTON_L(5));
  // m_DoubleSub = frc2::Trigger(BUTTON_L_TWO(13));
  // m_SingleSubCube = frc2::Trigger(BUTTON_L(7));

  // m_LeftGrid    = frc2::Trigger(BUTTON_L_TWO(LEFT_GRID));
  // m_CenterGrid  = frc2::Trigger(BUTTON_L_TWO(CENTER_GRID));
  // m_RightGrid   = frc2::Trigger(BUTTON_L_TWO(RIGHT_GRID));

  // m_MidCone = frc2::JoystickButton(BUTTON_L_TWO(TRANSIT_MODE));
	// m_MidCube = frc2::JoystickButton(BUTTON_L_TWO(GROUND_PICKUP_MODE));
  // m_PlacingMode = frc2::Trigger(BUTTON_L_TWO(PLACING_MODE));
  // m_GroundPickup = frc2::Trigger(BUTTON_L_TWO(GROUND_PICKUP_MODE));

 //m_NavXReset = frc2::Trigger(BUTTON_L(8)); //PUT Define
  // GetArm().m_PlacingMode = frc2::Trigger(BUTTON_L_TWO(15));
  //m_AutoBalance = frc2::Trigger(BUTTON_L(3));
  //m_Print = frc2::Trigger(BUTTON_L(2));

  // m_VisionPoseReset = frc2::Trigger([&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(6); }); //PUT Define
 
  // GetDriveTrain().m_TestJoystickButton.OnTrue(new frc2::InstantCommand([&]{
  //   GetVision().PrintValues();
  // }));

  // m_Print.whileTrue(
  //   new frc2::InstantCommand([&]{
  //     DebugOutF("StringDeg: " + std::to_string(GetArm().WristTicksToDegrees(GetArm().WristStringPotUnitsToTicks(GetArm().GetStringPot().GetValue()))));
  //     DebugOutF("PivotDeg: " + std::to_string(GetArm().PivotTicksToDegrees(GetArm().GetPivotMotor().GetSelectedSensorPosition())));
  //     DebugOutF("StringPotRaw: " + std::to_string(GetArm().GetStringPot().GetValue()));
  //   })
  // );


  // m_NavXReset.OnTrue(
  //   new frc2::InstantCommand([&]{
  //     DebugOutF("NavX Zero");
  //     zeroGyroscope();
  // }));
  
  // GetArm().m_PlacingMode.OnTrue(new frc2::ParallelCommandGroup(
	// 		PivotToPos(Robot::GetRobot()->GetArm().m_PivotMatrix[SelectedRow][SelectedColumn]), 
  //     WristToPos(Robot::GetRobot()->GetArm().m_WristMatrix[SelectedRow][SelectedColumn])
	// ));

  //m_TR.OnTrue(frc2::PrintCommand("Nothing").ToPtr());


  // m_AutoBalance.ToggleOnTrue(
  //   new frc2::InstantCommand([&]{
  //     new Shooter();
  //   })
  // );

  /*GetArm().m_PlacingMode.OnTrue(
      new frc2::InstantCommand([&]{
        if(m_ArmCommand != nullptr){
          m_ArmCommand->Cancel();
        }
        //Robot::GetRobot()->GetArm().m_PivotPos = Robot::GetRobot()->GetArm().m_PivotMatrix[SelectedRow][SelectedColumn];
        //Robot::GetRobot()->GetArm().m_WristPos = Robot::GetRobot()->GetArm().m_WristMatrix[SelectedRow][SelectedColumn];
        //m_ArmCommand = new frc2::ParallelCommandGroup(WristToPos(),
                                                      //PivotToPos(),
                                                      //frc2::PrintCommand("Execute")
        );
        m_ArmCommand->Schedule(); 
        // Robot::GetRobot()->GetArm().m_PivotPos = 0;
        // Robot::GetRobot()->GetArm().m_WristPos = 0;
      })
  );*/

//   m_GroundPickup.OnTrue(
//     new frc2::ParallelCommandGroup(
//       frc2::InstantCommand([&]{
//        // Robot::GetRobot()->GetArm().m_PivotPos = 94.0;
//         //Robot::GetRobot()->GetArm().m_WristPos = 10.0;
//         //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
//       }),
//       //WristToPos(),
//       //PivotToPos(),
//       frc2::PrintCommand("Command")
//     )
//   );

//   m_SingleSub.OnTrue(
//     new frc2::ParallelCommandGroup(
//       frc2::InstantCommand([&]{
//         //Robot::GetRobot()->GetArm().m_PivotPos = Robot::GetRobot()->GetArm().m_PivotMatrix[0][2];
//         //Robot::GetRobot()->GetArm().m_WristPos = Robot::GetRobot()->GetArm().m_WristMatrix[0][2];
//         //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL / 1.5, WRIST_DFLT_ACC / 2.0); //make the divodor a bit smaller 2 is really slow
//       }),
//       //WristToPos(),
//       //PivotToPos(),
//       frc2::PrintCommand("Command")
//     )
//   );

//   m_DoubleSub.OnTrue(
//     new frc2::ParallelCommandGroup(
//       frc2::InstantCommand([&]{
//         //Robot::GetRobot()->GetArm().m_PivotPos = Robot::GetRobot()->GetArm().m_PivotMatrix[0][0];
//         //Robot::GetRobot()->GetArm().m_WristPos = Robot::GetRobot()->GetArm().m_WristMatrix[0][0];   
//         //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 1, PIVOT_DFLT_ACC / PIVOT_ACC_DIVISOR, WRIST_DFLT_VEL, WRIST_DFLT_ACC); 
//       }),
//       //WristToPos(),
//       //PivotToPos(),
//       frc2::PrintCommand("Command")
//     )
//   );

//   m_SingleSubCube.OnTrue(
//       new frc2::ParallelCommandGroup(
//       frc2::InstantCommand([&]{
//        // Robot::GetRobot()->GetArm().m_PivotPos = 72.0;
//         //Robot::GetRobot()->GetArm().m_WristPos = 54.0; 
//         //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
//       }),
//       //WristToPos(),
//       //PivotToPos(),
//       frc2::PrintCommand("Command")
//     ));

//   m_ML.OnTrue(new frc2::InstantCommand([&]{
//     DebugOutF("m_ML");
//     //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 1, PIVOT_DFLT_ACC / PIVOT_ACC_DIVISOR, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
//     SelectedRow = 1;
//     SelectedColumn = std::abs(m_ColOffset - 0);
//     frc::Pose2d SelectedPose = 
// 		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 		Robot::GetRobot()->GetDriveTrain().m_TransformedPose =  (SelectedPose).TransformBy(frc::Transform2d(frc::Translation2d(0_m, 0_m), frc::Rotation2d(180_deg)));
        
//   }));


//   m_BL.OnTrue(new frc2::InstantCommand([&]{
//     DebugOutF("m_BL");
//     SelectedRow = 2;
//     SelectedColumn = std::abs(m_ColOffset - 0);
//     frc::Pose2d SelectedPose = 
// 		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
//   }));


//   m_TC.OnTrue(new frc2::InstantCommand([&]{
//     DebugOutF("m_TC");
//     //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 1, PIVOT_DFLT_ACC / PIVOT_ACC_DIVISOR, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
//     SelectedRow = 0;
//     SelectedColumn = std::abs(m_ColOffset - 1);
//     frc::Pose2d SelectedPose = 
// 		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose).TransformBy(frc::Transform2d(frc::Translation2d(0_m, 0_m), frc::Rotation2d(180_deg)));
//   }));


//   m_MC.OnTrue(new frc2::InstantCommand([&]{
//     DebugOutF("m_MC");
//     SelectedRow = 1;
//     SelectedColumn = std::abs(m_ColOffset - 1);
//     frc::Pose2d SelectedPose = 
// 		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
//   }));

//   m_BC.OnTrue(new frc2::InstantCommand([&]{
//     DebugOutF("m_BC");
//     SelectedRow = 2;
//     SelectedColumn = std::abs(m_ColOffset - 1);
//     frc::Pose2d SelectedPose = 
// 		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
//   }));


//   // m_TR.OnTrue(new frc2::InstantCommand([&]{
//   //   DebugOutF("m_TR - nothing");
//   //   // frc::Pose2d SelectedPose = 
// 	// 	// Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 	// 	// Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
//   // }));

//   m_MR.OnTrue(new frc2::InstantCommand([&]{
// 		DebugOutF("m_MR");
//     //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL / 1, PIVOT_DFLT_ACC / PIVOT_ACC_DIVISOR, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
//     SelectedRow = 1;
//     SelectedColumn = std::abs(m_ColOffset - 2);
// 		frc::Pose2d SelectedPose = 
// 		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose).TransformBy(frc::Transform2d(frc::Translation2d(0_m, 0_m), frc::Rotation2d(180_deg)));
//   }));

//   m_BR.OnTrue(new frc2::InstantCommand([&]{
// 		DebugOutF("m_BR");
//     SelectedRow = 2;
//     SelectedColumn = std::abs(m_ColOffset - 2);
// 		frc::Pose2d SelectedPose = 
// 		  Robot::GetRobot()->GetDriveTrain().m_PoseMatrix[SelectedRow][SelectedColumn];
// 		Robot::GetRobot()->GetDriveTrain().m_TransformedPose = TransformPose(SelectedPose);
//   }));

//   m_BigRed.OnTrue(
//     new frc2::ParallelCommandGroup(
//       frc2::InstantCommand([&]{
//         //Robot::GetRobot()->GetArm().m_PivotPos = 92.0;
//         //Robot::GetRobot()->GetArm().m_WristPos = 127.0;
//         //Robot::GetRobot()->GetArm().SetMotionMagicValues(PIVOT_DFLT_VEL, PIVOT_DFLT_ACC, WRIST_DFLT_VEL, WRIST_DFLT_ACC);
//         frc2::PrintCommand("Big Red");
//       }),
//       /*frc2::SequentialCommandGroup(
//         frc2::WaitCommand(0.25_s),
//         //PivotToPos()
//       ),   */   
//       //WristToPos(),
//       frc2::PrintCommand("Command")
//     )
// 	);

//   m_LeftGrid.OnTrue(new frc2::InstantCommand([&]{
//     DebugOutF("m_LeftGrid");
// 		if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)) {
//       Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 0;
// 		} else{
// 			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 2;
// 	}}));

//   m_CenterGrid.OnTrue(new frc2::InstantCommand([&]{
//     Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 1;
//   }));

//   m_RightGrid.OnTrue(new frc2::InstantCommand([&]{
// 		DebugOutF("m_RightGrid");
// 		if(COB_GET_ENTRY(COB_KEY_IS_RED).GetBoolean(false)){
// 			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 2;
// 		} else{
// 			Robot::GetRobot()->GetDriveTrain().m_SelectedGrid = 0;
// 	}}));

//   m_VisionPoseReset.OnTrue(
//     new frc2::InstantCommand([&] {
//       DebugOutF("Pose Resetting");
//       if(COB_GET_ENTRY(m_Vision.FrontBack("tv")).GetInteger(0) == 1 && COB_GET_ENTRY(m_Vision.FrontBack("botpose")).GetDoubleArray(std::span<double>()).size() != 0){
//         frc::Pose2d startingPose = frc::Pose2d(m_Vision.GetFieldPose().Translation(), units::radian_t(Deg2Rad(GetAngle())));
//         GetDriveTrain().GetOdometry()->ResetPosition(units::radian_t(Deg2Rad(GetAngle())), 
//         wpi::array<frc::SwerveModulePosition, 4>
//               (GetDriveTrain().m_FrontLeftModule.GetPosition(), GetDriveTrain().m_FrontRightModule.GetPosition(), GetDriveTrain().m_BackLeftModule.GetPosition(), GetDriveTrain().m_BackRightModule.GetPosition()), 
//         startingPose);
//         DebugOutF("Pose Reset. X: " + std::to_string(startingPose.X().value()) + ", Y: " + std::to_string(startingPose.Y().value()) + ", Z: " + std::to_string(startingPose.Rotation().Degrees().value()));
//       } else {
//         DebugOutF("Pose Reset Fail");
//       }
//     })
//   );
   }

frc2::CommandPtr Robot::getAutonomousCommand() {
  // Load the path you want to follow using its name in the GUI
  auto path = PathPlannerPath::fromPathFile(m_AutoPath);
  // // DebugOutF(std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
  // // DebugOutF(std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
  // // DebugOutF(std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
  // // Create a path following command using AutoBuilder. This will also trigger event markers.
  startingPose = path.get()->getStartingDifferentialPose();
  return AutoBuilder::followPath(path);
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
  // DebugOutF("Stringpot Value: " + std::to_string(GetArm().GetStringPot().GetValue()));
  // m_AutoPath = std::string(Robot::GetRobot()->GetCOB().GetTable().GetEntry("/COB/auto").GetString(""));
  // DebugOutF("Row: " + std::to_string(SelectedRow) + " , Col: " + std::to_string(SelectedColumn));


  // if(Robot::GetButtonBoard().GetRawButton(15)){
  //   DebugOutF("BL: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackLeftModule.GetSteerAngle())));
  //   DebugOutF("BR: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackRightModule.GetSteerAngle())));
  //   DebugOutF("FL: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontLeftModule.GetSteerAngle())));
  //   DebugOutF("FR: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontRightModule.GetSteerAngle())));
  // }

  // if(GetButtonBoard().GetRawButton(16)) {
    // DebugOutF("Stringpot Value: " + std::to_string(GetArm().GetStringPot().GetValue()));
    // DebugOutF("BL Voltage: " + std::to_string(GetDriveTrain().m_BackLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("BR Voltage: " + std::to_string(GetDriveTrain().m_BackRightModule.GetSteerSensorVoltage()));
    // DebugOutF("FL Voltage: " + std::to_string(GetDriveTrain().m_FrontLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("FR Voltage: " + std::to_string(GetDriveTrain().m_FrontRightModule.GetSteerSensorVoltage()));
    // DebugOutF("FL: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontLeftModule.GetSteerAngle())));
    // DebugOutF("FR: " + std::to_string(Rad2Deg(GetDriveTrain().m_FrontRightModule.GetSteerAngle())));
    // DebugOutF("BL: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackLeftModule.GetSteerAngle())));
    // DebugOutF("BR: " + std::to_string(Rad2Deg(GetDriveTrain().m_BackRightModule.GetSteerAngle())));
  // }

  // DebugOutF("feed to intake servo: " + std::to_string(GetRobot()->GetDriveTrain().m_DustpanRotate.Get()));
  // DebugOutF("upwards servo: " + std::to_string(GetRobot()->GetDriveTrain().m_DustpanLaunch.Get()));
  // DebugOutF("pivot: " + std::to_string(GetRobot()->GetArm().GetPivotMotor().Get()));
  // DebugOutF("shooter 1: " + std::to_string(GetRobot()->GetArm().m_ShooterMotor1.Get()) + "; shooter 2: " + std::to_string(GetRobot()->GetArm().m_ShooterMotor2.Get()));
  // DebugOutF("bottom intake motor: " + std::to_string(GetRobot()->GetArm().m_Feeder.GetMotorOutputPercent()));

  bool preset = false;
  double difference = 0;

  // if(GetButtonBoard().GetRawButton(7)) {
  //   preset = true;
  //   // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
  //   // GetArm().GetPivotMotor().SetControl(m_DutyCycleOutRequest.WithOutput(GetArm().m_OriginalPivotRotations - GetArm().PivotStringPotUnitsToRotations(GetArm().m_StringPotOffset)));
  //   // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
  //   // DebugOutF("stringpot value: " + std::to_string(GetArm().GetStringPot().GetValue()));
  //   // DebugOutF("preset value: " + CLOSEUPSHOOTSTRINGPOT);
  //   difference = GetRobot()->GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
  //   DebugOutF("difference: " + std::to_string(difference));
  //   if(GetRobot()->GetArm().GetStringPot().GetValue() != CLOSEUPSHOOTSTRINGPOT) {
  //     if(difference > 0) {
  //       GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3));
  //     } else if(difference < 0) {
  //       GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
  //     }
  //   }
  //   preset = false;
  // }

  // if(GetButtonBoard().GetRawButton(8)) {
  //   preset = true;
  //   // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - CLOSEUPSHOOTSTRINGPOT;
  //   // GetArm().GetPivotMotor().SetControl(m_DutyCycleOutRequest.WithOutput(GetArm().m_OriginalPivotRotations - GetArm().PivotStringPotUnitsToRotations(GetArm().m_StringPotOffset)));
  //   // GetArm().m_StringPotOffset = GetArm().GetStringPot().GetValue() - PICKUPSTRINGPOT;
  //   if(GetRobot()->GetArm().GetStringPot().GetValue() != PICKUPSTRINGPOT) {
  //     difference = GetRobot()->GetArm().GetStringPot().GetValue() - PICKUPSTRINGPOT;
  //     if(difference > 0) {
  //       GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3));
  //     } else if(difference < 0) {
  //       GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
  //     }
  //   }
  //   preset = false;
  // }

  // if(GetButtonBoard().GetRawButton(18)) {
  //   GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.3));
  // } else if(GetButtonBoard().GetRawButton(17) && GetArm().GetStringPot().GetValue() < 815) {
  //   GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.3));
  // } else if(!GetButtonBoard().GetRawButton(17) && !GetButtonBoard().GetRawButton(18) && !preset) {
  //   GetArm().GetPivotMotor().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  // }

  // if(GetButtonBoard().GetRawButton(21)){
  //   GetRobot()->GetDriveTrain().m_DustpanRotate.Set(0);
  // }
  
  // if(GetButtonBoard().GetRawButton(22)) {
  //   GetRobot()->GetDriveTrain().m_DustpanRotate.Set(1);
  // }

  // if(GetButtonBoard().GetRawButton(5)){
  //   GetRobot()->GetDriveTrain().m_DustpanLaunch.Set(0.75);
  // } else if(!GetButtonBoard().GetRawButton(5)) {
  //   GetRobot()->GetDriveTrain().m_DustpanLaunch.Set(1);
  // }

  // if(GetButtonBoard().GetRawButton(1)) {
  //   double output = GetButtonBoard().GetRawAxis(SHOOTER_SPEED);
  //   GetArm().GetShooterMotor1().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-output + 0.05));
  //   GetArm().GetShooterMotor2().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-output));
  // } else if(!GetButtonBoard().GetRawButton(1)) {
  //   GetArm().GetShooterMotor1().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  //   GetArm().GetShooterMotor2().SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  // }

  // if(GetButtonBoard().GetRawButton(2)) {
  //   GetArm().GetFeeder().Set(ControlMode::PercentOutput, 0.25);
  // } else if(!GetButtonBoard().GetRawButton(2)) {
  //   GetArm().GetFeeder().Set(ControlMode::PercentOutput, 0);
  // }



  if(Robot::GetButtonBoard().GetRawButton(16)){
    // DebugOutF("Stringpot Value: " + std::to_string(GetArm().GetStringPot().GetValue()));
    // DebugOutF("BL Voltage: " + std::to_string(GetDriveTrain().m_BackLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("BR Voltage: " + std::to_string(GetDriveTrain().m_BackRightModule.GetSteerSensorVoltage()));
    // DebugOutF("FL Voltage: " + std::to_string(GetDriveTrain().m_FrontLeftModule.GetSteerSensorVoltage()));
    // DebugOutF("FR Voltage: " + std::to_string(GetDriveTrain().m_FrontRightModule.GetSteerSensorVoltage()));
    // DebugOutF("Max Sensor Voltage: " + std::to_string(frc::RobotController::GetVoltage5V()));
  }

  //LED
  // m_LED.SponsorBoardAllianceColor();
  // //m_LED.LowBattery();
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
  GetDriveTrain().BreakMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void Robot::DisabledPeriodic() {

}


/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // MotorInversionCheck();
  m_AutoFlag = true;
  DebugOutF("Auto init");

  frc2::CommandScheduler::GetInstance().CancelAll();
  GetNavX().ZeroYaw();
  GetNavX().SetAngleAdjustment(0);
  GetDriveTrain().BreakMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  m_AutoPath = "part1Blue";
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
  );
}
void Robot::AutonomousPeriodic() {
    // DebugOutF("X: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().X().value()));
    // DebugOutF("Y: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Y().value()));
    // DebugOutF("Deg: " + std::to_string(GetDriveTrain().GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));
}

void Robot::TeleopInit() {
  MotorInversionCheck();
  // if (m_autonomousCommand) {
  //   m_autonomousCommand->Cancel();
  // }

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
  // m_DriveTrain.BreakMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
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
  MotorInversionCheck();
  // if(inversionPrint) {
  //   DebugOutF("print");
  //   MotorInversionCheck();
  //   DebugOutF("fixed");
  //   DebugOutF("print:");
  //   MotorInversionCheck();
  //   inversionPrint = false;
  // }
  //DebugOutF("Theta: " + std::to_string(GetAngle())); 
  
  // DebugOutF("LLX: " + std::to_string(m_Vision.GetPoseBlue().X().value()));
  // DebugOutF("LLY: " + std::to_string(m_Vision.GetPoseBlue().Y().value()));
  // DebugOutF("LLZ: " + std::to_string(m_Vision.GetPoseBlue().Rotation().Degrees().value()));
}

void Robot::MotorInversionCheck() {
  // DebugOutF("frd: " + std::to_string(GetDriveTrain().m_FrontRightModule.m_SteerController.motor.GetInverted()));
  // DebugOutF("frs: " + std::to_string(GetDriveTrain().m_FrontRightModule.m_SteerController.motor.GetInverted()));
  // DebugOutF("fld: " + std::to_string(GetDriveTrain().m_FrontLeftModule.m_DriveController.motor.GetInverted()));
  // DebugOutF("fls: " + std::to_string(GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.GetInverted()));
  // DebugOutF("brd: " + std::to_string(GetDriveTrain().m_BackRightModule.m_DriveController.motor.GetInverted()));
  // DebugOutF("brs: " + std::to_string(GetDriveTrain().m_BackRightModule.m_SteerController.motor.GetInverted()));
  // DebugOutF("bld: " + std::to_string(GetDriveTrain().m_BackLeftModule.m_DriveController.motor.GetInverted()));
  // DebugOutF("bls: " + std::to_string(GetDriveTrain().m_BackLeftModule.m_SteerController.motor.GetInverted()));
}

// void Robot::MotorInversionCorrection(ctre::phoenix6::hardware::TalonFX motor, int ID, bool invert) {

//   ctre::phoenix6::hardware::
//   if((motor.GetInverted() != invert)) {
//     if(ID == FRONT_RIGHT_MODULE_STEER_MOTOR || ID == FRONT_LEFT_MODULE_DRIVE_MOTOR || ID == BACK_LEFT_MODULE_STEER_MOTOR || ID == BACK_RIGHT_MODULE_DRIVE_MOTOR) {
//        motor.SetInverted(true);
//        DebugOutF("motor " + std::to_string(ID) + "was set to inverted");
//     } else {
//       motor.SetInverted(false);
//       DebugOutF("motor " + std::to_string(ID) + "was set to not inverted");
//     }
//   }
// }

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
