#include "subsystems/Drivetrain.h"
#include "Robot.h"

#include <frc/trajectory/Trajectory.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/Timer.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include "./commands/LockOn.h"
#include "./commands/Lock180.h"
#include "./commands/Strafe.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include "./commands/Flywheel.h"
#include "./commands/ConstantPivot.h"

using namespace pathplanner;

using namespace ctre::phoenix6;

//Constructor
DriveTrain::DriveTrain()
    : m_FrontLeftLocation(units::meter_t (DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (-DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_FrontRightLocation(units::meter_t (DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_BackLeftLocation(units::meter_t (-DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (-DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_BackRightLocation(units::meter_t (-DRIVETRAIN_TRACKWIDTH_METERS / 2.0), units::meter_t (DRIVETRAIN_WHEELBASE_METERS / 2.0)),
      m_Kinematics(m_FrontLeftLocation, m_FrontRightLocation, m_BackLeftLocation, m_BackRightLocation),
      m_Rotation(0_rad),
      driveState(0),
      m_Odometry(m_Kinematics, m_Rotation, ( wpi::array<frc::SwerveModulePosition, 4>
         (m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition())), frc::Pose2d(0_m, 0_m, 0_rad)),
      m_FrontLeftModule(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_ENCODER_PORT, FRONT_LEFT_MODULE_STEER_OFFSET),
      m_FrontRightModule(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_ENCODER_PORT, FRONT_RIGHT_MODULE_STEER_OFFSET),
      m_BackLeftModule(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_ENCODER_PORT, BACK_LEFT_MODULE_STEER_OFFSET),
      m_BackRightModule(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_ENCODER_PORT, BACK_RIGHT_MODULE_STEER_OFFSET),
      m_ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s},
      m_xController(0.75, 0.5, 0.15),
      m_yController(0.75, 0.5, 0.15),
      m_ThetaController(5.0, 4.0, 0.0, frc::TrapezoidProfile<units::radian>::Constraints{3.14_rad_per_s, (1/2) * 3.14_rad_per_s / 1_s}),
      // m_ThetaController(20.0, 0.0, 0.0, frc::TrapezoidProfile<units::radian>::Constraints{3.14_rad_per_s, (1/2) * 3.14_rad_per_s / 1_s}),
      m_HolonomicController(m_xController, m_yController, m_ThetaController),
      m_Climb(CLIMB_MOTOR),
      m_Lock180Button([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(1);}),
      m_LockOnButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(2);}),
      m_NavXResetButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(3);}),
      m_JoystickFlywheel([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(5);}),
      m_StrafeLeft([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(6);}),
      m_StrafeRight([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(4);}),
      m_ClimbRobotUp([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(CLIMB_UP);}),
	    m_ClimbRobotDown([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(CLIMB_DOWN);}),
      m_VisionAim([&] {return Robot::GetRobot()->GetButtonBoard().GetRawButton(AIM_BUTTON);}),
      m_Timer(),
      m_EventMap()
{
  // AutoBuilder::configureHolonomic(
  //       [this]() { return this->getPose(); }, // Robot pose supplier
  //       [this](frc::Pose2d pose){ this->resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
  //       [this]() { return this->getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //       [this](frc::ChassisSpeeds robotRelativeSpeeds){ this->DriveRobotRelative(robotRelativeSpeeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  //       HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
  //           PIDConstants(10.5, 0.0, 0.0), // Translation PID constants
  //           PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
  //           // kMAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
  //           units::meters_per_second_t(2.0),
  //           0.5298_m, // Drive base radius in meters. Distance from robot center to furthest module.
  //           ReplanningConfig() // Default path replanning config. See the API for the options here
  //       ),
  //       []() {
  //           // Boolean supplier that controls when the path will be mirrored for the red alliance
  //           // This will flip the path being followed to the red side of the field.
  //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  //           auto alliance = DriverStation::GetAlliance();
  //           if (alliance) {
  //               return alliance.value() == DriverStation::Alliance::kBlue;
  //           }
  //           return false;
  //       },
  //       this // Reference to this subsystem to set requirements
  //   );
}

void DriveTrain::DriveInit() {
  m_Rotation = frc::Rotation2d(units::radian_t(Robot::GetRobot()->GetAngle()));
  m_Climb.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  SetDefaultCommand(DriveWithJoystick());

  m_LockOnButton.ToggleOnTrue(new frc2::ParallelCommandGroup(
    LockOn(),
    ConstantPivot(),
    Flywheel()
  ));

  m_Lock180Button.ToggleOnTrue(new Lock180());

  m_StrafeLeft.WhileTrue(new Strafe(1));
  
  m_StrafeRight.WhileTrue(new Strafe(0));

  m_JoystickFlywheel.OnTrue(new frc2::InstantCommand([&] {
    Robot::GetRobot()->GetArm().GetDustpanLaunchServo().Set(0.75);
	})).OnFalse(new frc2::InstantCommand([&] {
		Robot::GetRobot()->GetArm().GetDustpanLaunchServo().Set(1);
	}));

  m_NavXResetButton.OnTrue(
    new frc2::InstantCommand([&]{
      DebugOutF("NavX Zero");
      Robot::GetRobot()->m_AngleOffset = 0;
      Robot::GetRobot()->zeroGyroscope();
      // Robot::GetRobot()->GetDriveTrain().GetOdometry()->ResetPosition()
  }));

  // m_Odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3U> {0.25, 0.25, .561799});

  m_ClimbRobotUp.OnTrue(new frc2::InstantCommand([&] {
    DebugOutF("climbing up on");
    m_Climb.Set(1);
    // m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0.5));
  })).OnFalse(new frc2::InstantCommand([&] {
    DebugOutF("climbing up off");
    m_Climb.Set(0);
    // m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  }));

  m_ClimbRobotDown.OnTrue(new frc2::InstantCommand([&] {
    DebugOutF("climbing down on");
    m_Climb.Set(-1);
    // m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(-0.5));
  })).OnFalse(new frc2::InstantCommand([&] {
    DebugOutF("climbing down off");
    m_Climb.Set(0);
    // m_Climb.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  }));
}

/*
Is called periodically
Passes module states to motors and updates odometry
*/
void DriveTrain::Periodic(){


  // m_StrafeLeft.WhileTrue(new BotStrafe(1));
  // m_StrafeRight.WhileTrue(new BotStrafe(2)); 
  // m_StrafeLeft.WhileFalse(new BotStrafe(0));
  // m_StrafeRight.WhileFalse(new BotStrafe(0));
  //m_StrafeLeft.OnTrue(new BotStrafe(1)).OnFalse(new BotStrafe(0));
  //m_StrafeRight.OnTrue(new BotStrafe(2)).OnFalse(new BotStrafe(0));

  // if (driveState = 1) {
  //   //DebugOutF("Drive state = 1");
  //   // Robot::GetRobot()->GetDriveTrain().m_FrontRightModule.Set(8.0, -90.0);
  //   // Robot::GetRobot()->GetDriveTrain().m_FrontLeftModule.Set(8.0, -90.0);
  //   // Robot::GetRobot()->GetDriveTrain().m_BackRightModule.Set(8.0, -90.0);
  //   // Robot::GetRobot()->GetDriveTrain().m_BackLeftModule.Set(8.0, -90.0);
  // } else if (driveState = 2) {
  //   //DebugOutF("Drive state = 2");
  //   // Robot::GetRobot()->GetDriveTrain().m_FrontRightModule.Set(8.0, 90.0);
  //   // Robot::GetRobot()->GetDriveTrain().m_FrontLeftModule.Set(8.0, 90.0);
  //   // Robot::GetRobot()->GetDriveTrain().m_BackRightModule.Set(8.0, 90.0);
  //   // Robot::GetRobot()->GetDriveTrain().m_BackLeftModule.Set(8.0, 90.0);
  // } else {
    //DebugOutF("Drive state = 0");
    if((m_ModuleStates[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE) == 0 && ((double) m_ModuleStates[0].angle.Radians() == 0)){
      m_FrontLeftModule.m_SteerController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
      m_FrontLeftModule.m_DriveController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    } else {
      m_FrontLeftModule.Set(m_ModuleStates[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE * -1, (double) m_ModuleStates[0].angle.Radians());
    }

    if((m_ModuleStates[1].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) m_ModuleStates[1].angle.Radians() == 0)){
      m_FrontRightModule.m_SteerController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
      m_FrontRightModule.m_DriveController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    } else {
      m_FrontRightModule.Set(m_ModuleStates[1].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) m_ModuleStates[1].angle.Radians() * -1.0);
    }

    if((m_ModuleStates[2].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) m_ModuleStates[2].angle.Radians() == 0)){
      m_BackLeftModule.m_SteerController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
      m_BackLeftModule.m_DriveController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    } else {
      m_BackLeftModule.Set(m_ModuleStates[2].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE, (double) m_ModuleStates[2].angle.Radians() * -1.0);
    }

    if((m_ModuleStates[3].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE == 0) && ((double) m_ModuleStates[3].angle.Radians() == 0)){
      m_BackRightModule.m_SteerController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
      m_BackRightModule.m_DriveController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    } else {
      m_BackRightModule.Set(m_ModuleStates[3].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE * -1, (double) m_ModuleStates[3].angle.Radians());
    }

  m_Rotation = frc::Rotation2d(units::angle::degree_t(Robot::GetRobot()->GetAngle()));

  m_ModulePositions = wpi::array<frc::SwerveModulePosition, 4>(m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition());

  m_Odometry.Update(m_Rotation, m_ModulePositions);

  // m_VisionPos = Robot::GetRobot()->GetVision().GetFieldPose();
  // m_VisionPosXArray.push_back(m_VisionPos.X().value());
  // m_VisionPosYArray.push_back(m_VisionPos.Y().value());
  // m_VisionPosTArray.push_back(m_VisionPos.Rotation().Degrees().value());

  // DebugOutF("xSD " + std::to_string(getStandardDeviation(m_VisionPosXArray)));
  // DebugOutF("ySD " + std::to_string(getStandardDeviation(m_VisionPosYArray)));
  // DebugOutF("tSD " + std::to_string(getStandardDeviation(m_VisionPosTArray)));

  // DebugOutF("OdoX: " + std::to_string(GetOdometry()->GetEstimatedPosition().X().value()));
  // DebugOutF("OdoY: " + std::to_string(GetOdometry()->GetEstimatedPosition().Y().value()));
  // DebugOutF("OdoZ: " + std::to_string(GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));

  // DebugOutF("visionX: " + std::to_string(Robot::GetRobot()->GetVision().GetFieldPose().X().value()));
  // DebugOutF("visionY: " + std::to_string(Robot::GetRobot()->GetVision().GetFieldPose().Y().value()));
  // DebugOutF("visionTheta: " + std::to_string(Robot::GetRobot()->GetVision().GetFieldPose().Rotation().Degrees().value()));

  // m_VisionRelative = Robot::GetRobot()->GetVision().GetFieldPose().RelativeTo(m_Odometry.GetEstimatedPosition());
  // if(Robot::GetRobot()->GetVision().GetLimeLight()->GetNumberArray("botpose_wpiblue", std::span<double>()).size() != 0) {
  //   if(m_VisionCounter == 25) {
  //     if(std::abs(m_VisionRelative.X().value()) < 1 && std::abs(m_VisionRelative.Y().value()) < 1 && std::abs(-fmod(360 - m_VisionRelative.Rotation().Degrees().value(), 360)) < 30) {
  //       m_Odometry.AddVisionMeasurement(frc::Pose2d(Robot::GetRobot()->GetVision().GetFieldPose().Translation(), m_Rotation), m_Timer.GetFPGATimestamp() - units::second_t((COB_GET_ENTRY(GET_VISION.FrontBack("tl")).GetDouble(0))/1000.0) - units::second_t((COB_GET_ENTRY(GET_VISION.FrontBack("cl")).GetDouble(0))/1000.0));
  //       DebugOutF("Vision Update");
  //       m_VisionCounter = 0;
  //     } else {
  //       m_VisionCounter++;
  //     }
  //   }
  // }
}
//Converts chassis speed object and updates module states
void DriveTrain::BaseDrive(frc::ChassisSpeeds chassisSpeeds) {
  m_ChassisSpeeds = frc::ChassisSpeeds::Discretize(chassisSpeeds, 0.02_s);
  auto [fl, fr, bl, br] = m_Kinematics.ToSwerveModuleStates(m_ChassisSpeeds);
  m_ModuleStates = {fl, fr, bl, br};
}

Pose2d DriveTrain::getPose() {
  return m_Odometry.GetEstimatedPosition();
}

void DriveTrain::resetPose(Pose2d pose) {
  m_Odometry.ResetPosition(Rotation2d(units::degree_t(Robot::GetRobot()->GetNavX().GetYaw())), m_ModulePositions, pose);
}

ChassisSpeeds DriveTrain::getRobotRelativeSpeeds() {
  Robot* r = Robot::GetRobot();
  return m_Kinematics.ToChassisSpeeds(m_ModuleStates);
}

void DriveTrain::DriveRobotRelative(frc::ChassisSpeeds robotRelativeSpeeds) {
  frc::ChassisSpeeds speeds = frc::ChassisSpeeds::Discretize(robotRelativeSpeeds, 0.02_s);

  auto swerveModuleStates = m_Kinematics.ToSwerveModuleStates(speeds);
  SetStates(swerveModuleStates);
}

void DriveTrain::SetStates(wpi::array<frc::SwerveModuleState, 4> states) {
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&states, kMAX_VELOCITY_METERS_PER_SECOND);

  m_FrontLeftModule.Set(states[0].speed.value() * -1, states[0].angle.Radians().value());
  m_FrontRightModule.Set(states[1].speed.value(), states[1].angle.Radians().value() * -1);
  m_BackLeftModule.Set(states[2].speed.value(), states[2].angle.Radians().value() * -1);
  m_BackRightModule.Set(states[3].speed.value() * -1, states[3].angle.Radians().value());
}

//Sets the drive motors to brake mode
void DriveTrain::BrakeMode(bool on) {
  m_FrontLeftModule.BrakeMode(on);
  m_FrontRightModule.BrakeMode(on);
  m_BackLeftModule.BrakeMode(on);
  m_BackRightModule.BrakeMode(on);
}

double DriveTrain::getStandardDeviation(std::vector<double> arr) {
  double sum;
  double mean;
  double sd;

  for(int i = 0; i < arr.size(); i++) {
    sum += arr.at(i);
  }
  mean = sum/arr.size();
  for(int i = 0; i < arr.size(); i++) {
    sd += pow(arr.at(i) - mean, 2);
  }
  sd = sqrt(sd / arr.size());
  return sd;
}
