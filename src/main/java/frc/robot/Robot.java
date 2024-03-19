// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.centerline.CenterLineManager;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private ShuffleboardTab tab;
  private RobotContainer m_robotContainer;
  private static Alliance allianceColor;
  Timer updateAllianceTimer;

  private AutoPhases currentAutoPhase;
  private CenterLineManager centerLineManager;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
        // Running on a real robot, log to a USB stick
        // Don't publish to Network Tables to reduce CPU usage
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        Logger.addDataReceiver(new RLOGServer(5800)); // for AdvantageScope
        break;

        // Running a physics simulator, log to local folder
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter(""));
        Logger.addDataReceiver(new RLOGServer(5800)); // for AdvantageScope
        Logger.addDataReceiver(new NT4Publisher()); // Network Tables
        break;

        // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit Logger
    Logger.start();

    updateAllianceTimer = new Timer();
    updateAllianceTimer.start();

    tab = Shuffleboard.getTab("Enabled Subsystems");

    subsystemEnabled("Drivebase", 1, 0, Constants.driveEnabled);

    subsystemEnabled("Joysticks", 0, 1, Constants.joysticksEnabled);
    subsystemEnabled("Gyro", 1, 1, Constants.gyroEnabled);
    subsystemEnabled("XBOX Controller", 4, 1, Constants.xboxEnabled);

    m_robotContainer = new RobotContainer();
  }

  // create new shuffleboard widget to show whether or not subsystem is enabled
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

  private enum AutoPhases {
    initialize,
    ourSide,
    centerLine,
    done
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.enableSubsystems();

    centerLineManager = new CenterLineManager(m_robotContainer.getCenterLineStrategy());

    m_autonomousCommand = m_robotContainer.getAutoInitialize();
    m_autonomousCommand.schedule();
    currentAutoPhase = AutoPhases.initialize;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if (Constants.Demo.inDemoMode) {
      return;
    }

    if (m_autonomousCommand != null && !m_autonomousCommand.isScheduled()) {
      switch (currentAutoPhase) {
        case initialize:
          m_autonomousCommand = m_robotContainer.getAutoOurSide();
          m_autonomousCommand.schedule();
          currentAutoPhase = AutoPhases.ourSide;
          break;
        case ourSide:
          if (centerLineManager.isDone()) {
            currentAutoPhase = AutoPhases.done;
          } else {
            m_autonomousCommand = centerLineManager.getCommand();
            m_autonomousCommand.schedule();
            currentAutoPhase = AutoPhases.centerLine;
          }
          break;
        case centerLine:
          if (centerLineManager.isDone()) {
            currentAutoPhase = AutoPhases.done;
          } else {
            m_autonomousCommand = centerLineManager.getCommand();
            m_autonomousCommand.schedule();
          }
          break;
        case done:
          break;
      }
    }
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
    m_robotContainer.teleopPeriodic();
  }

  /** This function is called once when test mode is enabled. */
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
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void updateAllianceColor() {
    try {
      allianceColor = DriverStation.getAlliance().get();
    } catch (NoSuchElementException e) {
      DriverStation.reportError("No Alliance Color", false);
      allianceColor = null;
    } catch (Exception e) {
      DriverStation.reportError("Update Alliance Error", false);
    }
  }

  public static Alliance getAllianceColor() {
    return allianceColor;
  }

  public static boolean isRed() {
    if (allianceColor == Alliance.Blue) {
      return false;
    }
    return true;
  }
}
