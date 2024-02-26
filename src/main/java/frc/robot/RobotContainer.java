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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.centerline.CenterLineManager.ScoringStrategy;
import frc.robot.commands.AutoIntakeDeploy;
import frc.robot.commands.AutoIntakeIn;
import frc.robot.commands.AutoSetOuttakeAdjust;
import frc.robot.commands.DriveManual.DriveManual;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualTrigger;
import frc.robot.commands.DriveStop;
import frc.robot.commands.EjectThroughOuttake;
import frc.robot.commands.IntakeManual;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.OuttakeAdjustToSpeaker;
import frc.robot.commands.OuttakeStop;
import frc.robot.commands.ResetFieldCentric;
import frc.robot.commands.SetRobotPose;
import frc.robot.commands.Shoot;
import frc.robot.commands.TunnelFeed;
import frc.robot.commands.TunnelStop;
import frc.robot.commands.WriteFiringSolutionAtCurrentPos;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.tunnel.Tunnel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Timer disableTimer = new Timer();

  // Define controllers
  public static CommandXboxController driveXbox;
  public static CommandXboxController operatorXbox;
  public static Joystick driveStick;
  public static Joystick rotateStick;

  private JoystickButton driveButtonSeven;
  private JoystickButton driveButtonTwelve;

  private final Drive drive = Drive.getInstance();
  private final Tunnel tunnel = Tunnel.getInstance();
  private final Outtake outtake = Outtake.getInstance();
  private final Intake intake = Intake.getInstance();

  private final WriteFiringSolutionAtCurrentPos writeFiringSolution =
      new WriteFiringSolutionAtCurrentPos();

  private final DriveManual driveManual = new DriveManual();

  private final TunnelFeed tunnelFeed = new TunnelFeed();

  private final OuttakeAdjustToSpeaker adjustOuttakeToSpeaker = new OuttakeAdjustToSpeaker();

  private final IntakeManual intakeManual = new IntakeManual();

  private final DriveStop driveStop = new DriveStop();

  private final IntakeStop intakeStop = new IntakeStop();

  private final OuttakeStop outtakeStop = new OuttakeStop();

  private final TunnelStop tunnelStop = new TunnelStop();

  private SendableChooser<String> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    PathPlannerManager.getInstance().addEvent("AutoIntakeDeploy", new AutoIntakeDeploy());
    PathPlannerManager.getInstance().addEvent("AutoIntakeIn", new AutoIntakeIn());
    PathPlannerManager.getInstance().addEvent("Shoot", new Shoot());

    // TODO: update speeds and angles
    PathPlannerManager.getInstance()
        .addEvent("SetOuttakeSubwooferBase", new AutoSetOuttakeAdjust(0, 0));
    PathPlannerManager.getInstance().addEvent("SetOuttakeN6", new AutoSetOuttakeAdjust(0, 0));
    PathPlannerManager.getInstance().addEvent("SetOuttakeN7", new AutoSetOuttakeAdjust(0, 0));
    PathPlannerManager.getInstance().addEvent("SetOuttakeN8", new AutoSetOuttakeAdjust(0, 0));
    PathPlannerManager.getInstance().addEvent("SetOuttakeTS", new AutoSetOuttakeAdjust(0, 0));
    PathPlannerManager.getInstance().addEvent("SetOuttakeMS", new AutoSetOuttakeAdjust(0, 0));
    PathPlannerManager.getInstance().addEvent("SetOuttakeBS", new AutoSetOuttakeAdjust(0, 0));

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", "None"); // DO NOT REMOVE THIS EVER
    autoChooser.addOption("3NoteToTopShoot", "3NoteToTopShoot");
    Shuffleboard.getTab("Autos").add(autoChooser).withPosition(0, 0).withSize(5, 2);

    FiringSolutionManager.getInstance().loadSolutions();

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManual);
    }

    if (Constants.tunnelEnabled) {
      tunnel.setDefaultCommand(tunnelFeed);
    }

    if (Constants.outtakeEnabled) {
      outtake.setDefaultCommand(adjustOuttakeToSpeaker);
    }

    if (Constants.intakeEnabled) {
      intake.setDefaultCommand(intakeManual);
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

      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonTwelve = new JoystickButton(driveStick, 12);

      driveButtonSeven.onTrue(new ResetFieldCentric(true));
      driveButtonTwelve.onTrue(driveStop);
    }

    if (Constants.xboxEnabled) {
      driveXbox = new CommandXboxController(2);
      operatorXbox = new CommandXboxController(3);
      driveXbox
          .leftBumper()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    driveManual.updateStateMachine(DriveManualTrigger.JOYSTICK_IN);
                  }));
      driveXbox.povUp().onTrue(new ResetFieldCentric(true));
      driveXbox.povRight().onTrue(writeFiringSolution);
      // Reset the odometry for testing speaker-centric driving. This assumes robot is on the
      // very left on the front of the speaker, facing down-field (forward).
      driveXbox
          .start()
          .onTrue(
              new SetRobotPose(
                  new Pose2d(1.3766260147094727, 5.414320468902588, new Rotation2d()), true));
      driveXbox.povDown().onTrue(driveStop);
      driveXbox
          .rightTrigger()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    RobotCoordinator.getInstance().setIntakeButtonState(true);
                  }));
      driveXbox
          .rightTrigger()
          .onFalse(
              Commands.runOnce(
                  () -> {
                    RobotCoordinator.getInstance().setIntakeButtonState(false);
                  }));
      driveXbox
          .rightBumper()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    RobotCoordinator.getInstance().setAutoIntakeButtonPressed(true);
                  }));

      driveXbox
          .rightBumper()
          .onFalse(
              Commands.runOnce(
                  () -> {
                    RobotCoordinator.getInstance().setAutoIntakeButtonPressed(false);
                  }));
      driveXbox.leftTrigger().whileTrue(new Shoot());
      operatorXbox.rightTrigger().whileTrue(new EjectThroughOuttake());
    }
  }

  public void disabledPeriodic() {
    if (disableTimer.hasElapsed(Constants.DriveConstants.disableBreakSec)) {
      if (Constants.driveEnabled) {
        drive.setCoastMode(); // robot has stopped, safe to enter coast mode
      }
      if (Constants.intakeEnabled) {
        intake.setCoastMode();
      }
      if (Constants.outtakeEnabled) {
        outtake.setCoastMode();
      }
      if (Constants.tunnelEnabled) {
        tunnel.setCoastMode();
      }
      disableTimer.stop();
      disableTimer.reset();
    }

    if (Constants.xboxEnabled) {
      // pressed when intake and outtake are in starting config
      // can only be pressed once after bootup
      driveXbox
          .povLeft()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    RobotCoordinator.getInstance().setInitAbsEncoderPressed(true);
                  }));
    }
  }

  public void enableSubsystems() {
    drive.setBrakeMode();
    tunnel.setBrakeMode();
    intake.setBrakeMode();
    outtake.setBrakeMode();
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    driveStop.schedule(); // interrupt all drive commands
    intakeStop.schedule(); // interrupt all intake commands
    outtakeStop.schedule(); // interrupt all outtake commands
    tunnelStop.schedule(); // interrupt all tunnel commands

    disableTimer.reset();
    disableTimer.start();
  }

  // Command that should always start off every auto
  public Command getAutoInitialize() {
    return new SequentialCommandGroup(new ResetFieldCentric(true));
  }

  // Command for the auto on our side of the field (PathPlanner Auto)
  public Command getAutoOurSide() {
    if (autoChooser.getSelected() == "None") {
      return Commands.none();
    } else {
      return PathPlannerManager.getInstance().buildAuto(autoChooser.getSelected());
    }
  }

  public ScoringStrategy getCenterLineStrategy() {
    return ScoringStrategy.DoNothing;
  }
}
