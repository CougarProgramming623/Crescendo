// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/simulation/DifferentialDrivetrainSim.h>

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

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>



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

  frc::SmartDashboard::PutData("Field", &m_field);

  m_field.SetRobotPose(GetVision().GetFieldPose());
}

void Robot::AutoButtons() {
  
   }

frc2::CommandPtr Robot::getAutonomousCommand() {
  // Load the path you want to follow using its name in the GUI
  auto path = PathPlannerPath::fromPathFile(m_AutoPath);
  
  startingPose = path.get()->getStartingDifferentialPose();
  return AutoBuilder::followPath(path);
}


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
  


  

  bool preset = false;
  double difference = 0;

  



  if(Robot::GetButtonBoard().GetRawButton(16)){
    
  }


                                                                                                                                  
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
  
  GetNavX().ZeroYaw();
  // m_DriveTrain.BreakMode(true);
  GetDriveTrain().m_BackLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_BackRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontLeftModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetDriveTrain().m_FrontRightModule.m_SteerController.motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  GetNavX().SetAngleAdjustment(0);
   
  
}

/**
 * This function is called periodically during operator control.  
 */
void Robot::TeleopPeriodic() {
  MotorInversionCheck();
  
}

void Robot::MotorInversionCheck() {
  
}


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
