// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final PowerDistribution PDH = new PowerDistribution();
  private Command m_autonomousCommand;
  private ShuffleboardTab tab;
  private ShuffleboardTab PDHTab;
  private RobotContainer m_robotContainer;
  private static Alliance allianceColor = Alliance.Invalid;
  Timer updateAllianceTimer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    // Record metadata
    
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      // Running on a real robot, log to a USB stick
      // Don't publish to Network Tables to reduce CPU usage
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        logger.addDataReceiver(new RLOGServer(5800));  // for AdvantageScope
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        logger.addDataReceiver(new WPILOGWriter(""));
        logger.addDataReceiver(new RLOGServer(5800));  // for AdvantageScope
        logger.addDataReceiver(new NT4Publisher());  // Network Tables
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit logger
    logger.start();

    updateAllianceTimer = new Timer();
    updateAllianceTimer.start();
    

    tab = Shuffleboard.getTab("Enabled Subsystems");
    PDHTab = Shuffleboard.getTab("PDH Currents");

    
    subsystemEnabled("Drivebase", 1, 0, Constants.driveEnabled);

    subsystemEnabled("Joysticks", 0, 1, Constants.joysticksEnabled);
    subsystemEnabled("Gyro", 1, 1, Constants.gyroEnabled);
    subsystemEnabled("XBOX Controller", 4, 1, Constants.xboxEnabled);

    m_robotContainer = new RobotContainer();
  }

    // create new shuffleboard tab to show whether or not subsystem is enabled
  // also print error to driver station if not
  private void subsystemEnabled(String title, int posX, int posY, boolean enabled) {
    tab.add(title, enabled)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(posX, posY)
    .withSize(1, 1);

    if (!enabled) {
      DriverStation.reportError(title + " not enabled", false);
    }
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (updateAllianceTimer.get() > 1) {
      updateAllianceColor();
      updateAllianceTimer.stop();
      updateAllianceTimer.reset();
      updateAllianceTimer.start();
    }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableSubsystems();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.enableSubsystems();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_robotContainer.enableSubsystems();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private void updateAllianceColor() {
    Alliance temp = DriverStation.getAlliance();
    if ((temp == Alliance.Red || temp == Alliance.Blue)) {
      allianceColor = temp;
    }
  }

  public static Alliance getAllianceColor() {
    return allianceColor;
  }
  
}
