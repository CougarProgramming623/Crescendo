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
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>

//#include "subsystems/Simulation.h"
#include <frc/simulation/DifferentialDrivetrainSim.h>

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
      // m_ModulePositions( wpi::array<frc::SwerveModulePosition, 4>
      //    (m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition())),
      m_Odometry(m_Kinematics, m_Rotation, ( wpi::array<frc::SwerveModulePosition, 4>
         (m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition())), frc::Pose2d(0_m, 0_m, 0_rad)),
      m_FrontLeftModule(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_ENCODER_PORT, FRONT_LEFT_MODULE_STEER_OFFSET),
      m_FrontRightModule(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_ENCODER_PORT, FRONT_RIGHT_MODULE_STEER_OFFSET),
      m_BackLeftModule(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_ENCODER_PORT, BACK_LEFT_MODULE_STEER_OFFSET),
      m_BackRightModule(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_ENCODER_PORT, BACK_RIGHT_MODULE_STEER_OFFSET),
      m_ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s}, 
      m_xController(0.7, 0.4, 0.3),
      m_yController(0.7, 0.4, 0.3),
      m_ThetaController(12, 0, .1, frc::TrapezoidProfile<units::radian>::Constraints{3.14_rad_per_s, (1/2) * 3.14_rad_per_s / 1_s}),
      m_HolonomicController(m_xController, m_yController, m_ThetaController),
      m_TestJoystickButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(1);}),
      m_JoystickButtonTwo([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(2);}),
      m_NavXResetButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(3);}),
      m_DuaLMotorControlButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(5);}),
      m_JoystickOuttake([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(6);}),
      m_ExtraJoystickButton([&] {return Robot::GetRobot()->GetJoyStick().GetRawButton(4);}),
      m_Timer(),
      m_EventMap()
{
  // AutoBuilder::configureHolonomic(
  //       [this](){ return getPose(); }, // Robot pose supplier
  //       [this](frc::Pose2d pose){ resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
  //       [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
  //       [this](frc::ChassisSpeeds speeds){ BaseDrive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
  //       HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
  //           PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
  //           PIDConstants(5.0, 0.0, 20.0), // Rotation PID constants
  //           kMAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
  //           0.871_m, // Drive base radius in meters. Distance from robot center to furthest module.
  //           ReplanningConfig() // Default path replanning config. See the API for the options here
  //       ),
  //       []() {
  //           // Boolean supplier that controls when the path will be mirrored for the red alliance
  //           // This will flip the path being followed to the red side of the field.
  //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

  //           auto alliance = DriverStation::GetAlliance();
  //           if (alliance) {
  //               return alliance.value() == DriverStation::Alliance::kRed;
  //           }
  //           return false;
  //       },
  //       this // Reference to this subsystem to set requirements
  //   );
  AutoBuilder::configureHolonomic(
        [this]() { return getPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]() { return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ BaseDrive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 20.0), // Rotation PID constants
            kMAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
            0.871_m, // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

 /*frc::sim::DifferentialDrivetrainSim m_driveSim{
      frc::DCMotor::Falcon500(2), // 2 NEO motors on each side of the drivetrain.
      7.29,               // 7.29:1 gearing reduction.
      7.5_kg_sq_m,        // MOI of 7.5 kg m^2 (from CAD model).
      60_kg,              // The mass of the robot is 60 kg.
      3_in,               // The robot uses 3" radius wheels.
      0.7112_m,           // The track width is 0.7112 meters.

      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};*/
}

void DriveTrain::DriveInit(){
  //m_Rotation = frc::Rotation2d(units::radian_t(Robot::GetRobot()->GetNavX().GetAngle()));
  SetDefaultCommand(DriveWithJoystick());

  // m_JoystickButtonTwo.ToggleOnTrue(new LockOn());

  //m_ExtraJoystickButton.WhileHeld(new DriveToPosCommand());

  m_NavXResetButton.OnTrue(
    new frc2::InstantCommand([&]{
      DebugOutF("NavX Zero");
      Robot::GetRobot()->zeroGyroscope();
  }));

  m_JoystickOuttake.WhileTrue(
    new frc2::InstantCommand([&]{
      // if(Robot::GetRobot()->m_Intake.GetCurrentCommand() != nullptr){
      //   Robot::GetRobot()->m_Intake.GetCurrentCommand()->Cancel();
      // }
      DebugOutF("Joystick Outtake");
      // Robot::GetRobot()->GetArm().GetBottomIntakeMotor().SetControl(Robot::GetRobot()->m_DutyCycleRequest.WithOutput(0.8));
      //Robot::GetRobot()->GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, .8);
    }
  ));

  // m_JoystickOuttake.OnFalse(
  //   new frc2::InstantCommand([&]{
  //     // Robot::GetRobot()->GetArm().GetBottomIntakeMotor().SetControl(Robot::GetRobot()->m_DutyCycleRequest.WithOutput(0));
  //     //Robot::GetRobot()->GetArm().GetBottomIntakeMotor().Set(ControlMode::PercentOutput, 0);
  //     frc2::CommandScheduler::GetInstance().Schedule(new DynamicIntake());
  //   })
  // );


  //m_DualMotorControlButton.ToggleOnTrue(new DualMotorControl());


  m_Odometry.SetVisionMeasurementStdDevs(wpi::array<double, 3U> {0.25, 0.25, .561799});
  // DebugOutF("frd: " + std::to_string(m_FrontRightModule.m_DriveController.motor.GetInverted()));
  // DebugOutF("frs: " + std::to_string(m_FrontRightModule.m_SteerController.motor.GetInverted()));
  // DebugOutF("fld: " + std::to_string(m_FrontLeftModule.m_DriveController.motor.GetInverted()));
  // DebugOutF("fls: " + std::to_string(m_FrontLeftModule.m_SteerController.motor.GetInverted()));
  // DebugOutF("brd: " + std::to_string(m_BackRightModule.m_DriveController.motor.GetInverted()));
  // DebugOutF("brs: " + std::to_string(m_BackRightModule.m_SteerController.motor.GetInverted()));
  // DebugOutF("bld: " + std::to_string(m_BackLeftModule.m_DriveController.motor.GetInverted()));
  // DebugOutF("bls: " + std::to_string(m_BackLeftModule.m_SteerController.motor.GetInverted()));
}

/*
Is called periodically
Passes module states to motors and updates odometry
*/
void DriveTrain::Periodic(){
  if((m_ModuleStates[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE) == 0 && ((double) m_ModuleStates[0].angle.Radians() == 0)){
    m_FrontLeftModule.m_SteerController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
    m_FrontLeftModule.m_DriveController.motor.SetControl(Robot::GetRobot()->m_DutyCycleOutRequest.WithOutput(0));
  } else {
    // DebugOutF("steer desired angle in radians: " + std::to_string((double)m_ModuleStates[0].angle.Radians()));
    // DebugOutF("drive desired speed in meters per second: " + std::to_string((m_ModuleStates[0].speed / kMAX_VELOCITY_METERS_PER_SECOND * kMAX_VOLTAGE).value()));
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

  m_Rotation = frc::Rotation2d(units::radian_t(Deg2Rad(Robot::GetRobot()->GetAngle())));

  m_ModulePositions = wpi::array<frc::SwerveModulePosition, 4>(m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(), m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition());


  m_VisionRelative = Robot::GetRobot()->GetVision().GetFieldPose().RelativeTo(m_Odometry.GetEstimatedPosition());
  //Robot::GetRobot()->GetVision().relativeDistancex(); useless idk why i did this
  //DebugOutF("OdoX: " + std::to_string(GetOdometry()->GetEstimatedPosition().X().value()));
  //DebugOutF("OdoY: " + std::to_string(GetOdometry()->GetEstimatedPosition().Y().value()));
  //DebugOutF("OdoZ: " + std::to_string(GetOdometry()->GetEstimatedPosition().Rotation().Degrees().value()));

  // DebugOutF("visionX: " + std::to_string(Robot::GetRobot()->GetVision().GetFieldPose().X().value()));
  // DebugOutF("visionY: " + std::to_string(Robot::GetRobot()->GetVision().GetFieldPose().Y().value()));
  // DebugOutF("visionTheta: " + std::to_string(Robot::GetRobot()->GetVision().GetFieldPose().Rotation().Degrees().value()));


  // if(COB_GET_ENTRY(GET_VISION.FrontBack("botpose")).GetDoubleArray(std::span<double>()).size() != 0){ // FIX uncomment when we have both limelights back
  // // if(COB_GET_ENTRY("/limelight/botpose").GetDoubleArray(std::span<double>()).size() != 0){ //Works with one limelight
  //   if((m_DriveToPoseFlag != true || m_VisionCounter == 25) && !Robot::GetRobot()->m_AutoFlag)
  //   {
  //     if(
  //      std::abs(m_VisionRelative.X().value()) < 1 &&
  //       std::abs(m_VisionRelative.Y().value()) < 1 &&
  //       std::abs(-fmod(360 - m_VisionRelative.Rotation().Degrees().value(), 360)) < 30) 
  //       {
  //         m_Odometry.AddVisionMeasurement(frc::Pose2d(Robot::GetRobot()->GetVision().GetFieldPose().Translation(), m_Rotation), m_Timer.GetFPGATimestamp()
  //         - units::second_t((COB_GET_ENTRY(GET_VISION.FrontBack("tl")).GetDouble(0))/1000.0) - units::second_t((COB_GET_ENTRY(GET_VISION.FrontBack("cl")).GetDouble(0))/1000.0));
  //         DebugOutF("Vision Update");
  //         m_VisionCounter = 0;
  //       } 
  //  } else { m_VisionCounter++; }
  //   m_Odometry.Update(m_Rotation, m_ModulePositions);
  // }
  frc::SmartDashboard::PutData("Theta PID Controller", &m_ThetaController);

}

/*void DriveTrain::SimulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
  m_driveSim.SetInputs(
    m_leftMotor.get() * units::volt_t(frc::RobotController::GetInputVoltage()),
    m_rightMotor.get() * units::volt_t(frc::RobotController::GetInputVoltage()));

  // Advance the model by 20 ms. Note that if you are running this
  // subsystem in a separate thread or have changed the nominal timestep
  // of TimedRobot, this value needs to match it.
  m_driveSim.Update(20_ms);

  // Update all of our sensors.
  m_leftEncoderSim.SetDistance(m_driveSim.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_driveSim.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(m_driveSim.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_driveSim.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_driveSim.GetHeading().Degrees());
}*/

//Converts chassis speed object and updates module states
void DriveTrain::BaseDrive(frc::ChassisSpeeds chassisSpeeds){
  m_ChassisSpeeds = chassisSpeeds;
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
  return ChassisSpeeds::FromRobotRelativeSpeeds(
      units::meters_per_second_t(0.1 * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //y
      units::meters_per_second_t(-0.1 * r->GetDriveTrain().kMAX_VELOCITY_METERS_PER_SECOND), //x
      units::radians_per_second_t(0.1 * r->GetDriveTrain().kMAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND), //rotation
      frc::Rotation2d(units::radian_t(Deg2Rad(-fmod(360 - r->GetNavX().GetAngle(), 360)))));
}

//Sets breakmode
void DriveTrain::BreakMode(bool on){
  m_FrontLeftModule.BreakMode(on);
  m_FrontRightModule.BreakMode(on);
  m_BackLeftModule.BreakMode(on);
  m_BackRightModule.BreakMode(on);
 }
