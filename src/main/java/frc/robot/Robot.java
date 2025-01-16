// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import swervelib.SwerveModule;

import swervelib.SwerveDriveTest;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private SwerveSubsystem m_SwerveSubsystem;

  private Timer disabledTimer;



  // public final SysIdRoutine l_sysIdRoutine =
  //   new SysIdRoutine(
  //       new SysIdRoutine.Config(
  //         null,        // Use default ramp rate (1 V/s)
  //         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
  //         null,        // Use default timeout (10 s)
  //                       // Log state with Phoenix SignalLogger class
  //         (state) -> SignalLogger.writeString("state", state.toString())
  //       ),
  //       new SysIdRoutine.Mechanism(
  //         (volts) -> {bl_motor.setControl(bl_voltReq.withOutput(volts.in(Volts)));
  //                     br_motor.setControl(br_voltReq.withOutput(volts.in(Volts)));},
          
  //         null,
  //         this
  //       )
  //   );

  //   public final SysIdRoutine r_sysIdRoutine =
  //   new SysIdRoutine(
  //       new SysIdRoutine.Config(
  //         null,        // Use default ramp rate (1 V/s)
  //         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
  //         null,        // Use default timeout (10 s)
  //                       // Log state with Phoenix SignalLogger class
  //         (state) -> SignalLogger.writeString("state", state.toString())
  //       ),
  //       new SysIdRoutine.Mechanism(
  //         (volts) -> {fl_motor.setControl(bl_voltReq.withOutput(volts.in(Volts)));
  //                     fr_motor.setControl(br_voltReq.withOutput(volts.in(Volts)));},
          
  //         null,
  //         this
  //       )
  //   );





  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    for (SwerveModule module : m_robotContainer.getDriveBase().getSwerveDriveConfiguration().modules) {
      System.out.println(module.getAbsolutePosition());
    }
    
    // m_robotContainer.getDriveBase().getSwerveDriveConfiguration().
    // m_robotContainer.getDriveBase().getSwerveDriveConfiguration().modules[0].getAbsolutePosition();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    System.out.println(("FL Voltage: " + m_robotContainer.getDriveBase().getSwerveDriveConfiguration().modules[0].getAbsolutePosition()));
    System.out.println(("FR Voltage: " + m_robotContainer.getDriveBase().getSwerveDriveConfiguration().modules[1].getAbsolutePosition()));
    System.out.println(("BL Voltage: " + m_robotContainer.getDriveBase().getSwerveDriveConfiguration().modules[2].getAbsolutePosition()));
    System.out.println(("BR Voltage: " + m_robotContainer.getDriveBase().getSwerveDriveConfiguration().modules[3].getAbsolutePosition()));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    // disabledTimer.reset();
    // disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.getDriveBase().zeroGyro();
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {

    m_robotContainer.getLeftSysID().onTrue(SwerveDriveTest.generateSysIdCommand(m_SwerveSubsystem.getSysIdRoutineLeft(), 2.0, 2.0, 2.0));


  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setDriveMode();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
