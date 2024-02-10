// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CenterLine.ScoreCenterLine;
import frc.robot.commands.CenterLine.ScoreCenterLine.ScoringStrategy;
import frc.robot.commands.DriveManual.DriveManual;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualTrigger;
import frc.robot.commands.DriveStop;
import frc.robot.commands.ResetFieldCentric;
import frc.robot.commands.SetRobotPose;
import frc.robot.commands.TunnelFeed;
import frc.robot.commands.TunnelStop;
import frc.robot.subsystems.TestRobotCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveInterface;
import frc.robot.subsystems.tunnel.Tunnel;
import frc.robot.subsystems.tunnel.TunnelInterface;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Timer disableTimer = new Timer();

  // Define controllers
  public static CommandXboxController xbox;
  public static Joystick driveStick;
  public static Joystick rotateStick;

  private JoystickButton driveButtonThree;
  private JoystickButton driveButtonSeven;
  private JoystickButton driveButtonTwelve;

  private final DriveInterface drive = new Drive();
  private final TunnelInterface tunnel = new Tunnel();

  private final DriveManual driveManual = new DriveManual(drive);
  private final DriveStop driveStop = new DriveStop(drive);

  private final TunnelFeed tunnelFeedDefault = new TunnelFeed(tunnel);
  private final TunnelStop tunnelStop = new TunnelStop(tunnel);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
        // Real robot, instantiate hardware IO implementations
      case REAL:
        break;

        // Sim robot, instantiate physics sim IO implementations
      case SIM:
        break;

        // Replayed robot, disable hardware IO implementations
      case REPLAY:
        break;
    }

    configureButtonBindings();

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManual);
    }

    if (Constants.tunnelEnabled) {
      tunnel.setDefaultCommand(tunnelFeedDefault);
    }
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);

      driveButtonThree = new JoystickButton(driveStick, 3);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonTwelve = new JoystickButton(driveStick, 12);

      driveButtonSeven.onTrue(new ResetFieldCentric(drive, true));
      driveButtonTwelve.onTrue(driveStop);
    }

    if (Constants.xboxEnabled) {
      xbox = new CommandXboxController(2);
      xbox.x()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    driveManual.updateStateMachine(DriveManualTrigger.JOYSTICK_IN);
                  }));
      xbox.povUp().onTrue(new ResetFieldCentric(drive, true));
      // Reset the odometry for testing speaker-centric driving. This assumes robot is on the
      // very left on the front of the speaker, facing down-field (forward).
      xbox.start()
          .onTrue(
              new SetRobotPose(
                  drive,
                  new Pose2d(1.3766260147094727, 5.414320468902588, new Rotation2d()),
                  true));
      xbox.povDown().onTrue(driveStop);
    }
  }

  public void disabledPeriodic() {
    // update logs

    if (disableTimer.hasElapsed(Constants.DriveConstants.disableBreakSec)) {
      if (Constants.driveEnabled) {
        drive.setCoastMode(); // robot has stopped, safe to enter coast mode
      }
      disableTimer.stop();
      disableTimer.reset();
    }
  }

  public void enableSubsystems() {
    drive.setBrakeMode();
    tunnel.setBrakeMode();
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    driveStop.schedule(); // interrupt all drive commands

    disableTimer.reset();
    disableTimer.start();
  }

  public Command getAutonomousCommand() {
    if (Constants.Demo.inDemoMode) {
      return null;
    }
    return new ScoreCenterLine(
        drive,
        new TestRobotCoordinator(true, true, true, true, true, true),
        ScoringStrategy.OneToFive);
  }

  // AUTO COMMANDS

  // Command that should always start off every auto
  public Command getAutoInitialize() {
    return new SequentialCommandGroup(new ResetFieldCentric(drive, true));
  }
}
