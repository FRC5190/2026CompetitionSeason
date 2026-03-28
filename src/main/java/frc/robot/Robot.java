// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.sim.RebuiltShooterSimHarness;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final DashboardPublisher dashboardPublisher;
  private RebuiltShooterSimHarness shooterSimHarness;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    dashboardPublisher = new DashboardPublisher(m_robotContainer);
  }

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    dashboardPublisher.update();

    // ChassisSpeeds speeds = m_robotContainer.drivebase.getSwerveDrive().getRobotVelocity();
    // double omegaRadPerSec = speeds.omegaRadiansPerSecond;
    // double omegaRps = omegaRadPerSec / (2.0 * Math.PI); // rotations per second
    // var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    // if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
    // m_robotContainer.drivebase.resetOdometry(llMeasurement.pose);
    // }

    // double tx = LimelightHelpers.getTX(kLL);
    // boolean tv = LimelightHelpers.getTV(kLL);
    // double ty = LimelightHelpers.getTY(kLL);
    // double ta = LimelightHelpers.getTA(kLL);
    // double tagId = LimelightHelpers.getFiducialID(kLL);

    // SmartDashboard.putNumber("LL/tx", tx);
    // SmartDashboard.putNumber("LL/ty", ty);
    // SmartDashboard.putNumber("LL/ta", ta);
    // SmartDashboard.putNumber("LL/tagID", tagId);

    // double now = Timer.getFPGATimestamp();
    // if (now - lastPrint > 0.5){
    // System.out.println("tv = " + tv + " tx = " + tx + " tagID = " + tagId);
    // lastPrint = now;
    // }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // System.out.println("SeesTag(10): " + drivebase.seesTag(10) + " TV: " +
    // LimelightHelpers.getTV("limelight"));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    shooterSimHarness = new RebuiltShooterSimHarness();
    shooterSimHarness.simulationInit();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (shooterSimHarness != null) {
      shooterSimHarness.simulationPeriodic();
    }
  }
}
