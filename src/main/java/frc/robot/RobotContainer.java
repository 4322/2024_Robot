// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AutoHelper.Auto;
import frc.robot.centerline.CenterLineManager.CenterLineScoringStrategy;
import frc.robot.commands.AtHome;
import frc.robot.commands.AutoIntakeDeploy;
import frc.robot.commands.AutoIntakeIn;
import frc.robot.commands.AutoSetOuttakeAdjust;
import frc.robot.commands.ClimberExtend;
import frc.robot.commands.ClimberRetract;
import frc.robot.commands.DriveManual.DriveManual;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualTrigger;
import frc.robot.commands.DriveStop;
import frc.robot.commands.EjectThroughIntake;
import frc.robot.commands.IntakeManual;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.OuttakeManual.OuttakeManual;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualTrigger;
import frc.robot.commands.OuttakeStop;
import frc.robot.commands.ResetFieldCentric;
import frc.robot.commands.SetPivotsBrakeMode;
import frc.robot.commands.SetPivotsCoastMode;
import frc.robot.commands.Shoot;
import frc.robot.commands.TunnelFeed;
import frc.robot.commands.TunnelStop;
import frc.robot.commands.UpdateOdometry;
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

  private boolean onOpponentFieldSide;

  // Need to instantiate RobotCoordinator first due to a bug in the WPI command library.
  // If it gets instantiated from a subsystem periodic method, we get a concurrency
  // exception in the command scheduler.
  private final RobotCoordinator robotCoordinator = RobotCoordinator.getInstance();
  private final Drive drive = Drive.getInstance();
  private final Tunnel tunnel = Tunnel.getInstance();
  private final Outtake outtake = Outtake.getInstance();
  private final Intake intake = Intake.getInstance();

  private final WriteFiringSolutionAtCurrentPos writeFiringSolution =
      new WriteFiringSolutionAtCurrentPos();

  private final DriveManual driveManual = new DriveManual();

  private final TunnelFeed tunnelFeed = new TunnelFeed();

  private final OuttakeManual outtakeManual = new OuttakeManual();

  private final IntakeManual intakeManual = new IntakeManual();

  private final DriveStop driveStop = new DriveStop();

  private final IntakeStop intakeStop = new IntakeStop();

  private final OuttakeStop outtakeStop = new OuttakeStop();

  private final TunnelStop tunnelStop = new TunnelStop();

  private final SendableChooser<Auto> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    PathPlannerManager.getInstance().addEvent("AutoIntakeDeploy", new AutoIntakeDeploy());
    PathPlannerManager.getInstance().addEvent("AutoIntakeIn", new AutoIntakeIn());
    PathPlannerManager.getInstance().addEvent("Shoot", new Shoot());

    PathPlannerManager.getInstance()
        .addEvent(
            "SetOuttakeSubwooferBase",
            new AutoSetOuttakeAdjust(Constants.FiringSolutions.SubwooferBase));
    PathPlannerManager.getInstance()
        .addEvent("SetOuttakeN6", new AutoSetOuttakeAdjust(Constants.FiringSolutions.N6));
    PathPlannerManager.getInstance()
        .addEvent("SetOuttakeN7", new AutoSetOuttakeAdjust(Constants.FiringSolutions.N7));
    PathPlannerManager.getInstance()
        .addEvent("SetOuttakeN8", new AutoSetOuttakeAdjust(Constants.FiringSolutions.N8));
    PathPlannerManager.getInstance()
        .addEvent("SetOuttakeTS", new AutoSetOuttakeAdjust(Constants.FiringSolutions.TS));
    PathPlannerManager.getInstance()
        .addEvent("SetOuttakeMS", new AutoSetOuttakeAdjust(Constants.FiringSolutions.MS));
    PathPlannerManager.getInstance()
        .addEvent("SetOuttakeBS", new AutoSetOuttakeAdjust(Constants.FiringSolutions.BS));

    autoChooser = new SendableChooser<>();
    AutoHelper.configAutoChooser(autoChooser);
    Shuffleboard.getTab("Autos").add(autoChooser).withPosition(0, 0).withSize(5, 2);

    FiringSolutionManager.getInstance().loadSolutions();

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManual);
    }

    if (Constants.tunnelEnabled) {
      tunnel.setDefaultCommand(tunnelFeed);
    }

    if (Constants.outtakeEnabled && !Constants.outtakeTuningMode) {
      outtake.setDefaultCommand(outtakeManual);
    }

    if (Constants.intakeEnabled) {
      intake.setDefaultCommand(intakeManual);
    }

    CommandScheduler.getInstance().schedule(new UpdateOdometry());
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
                    driveManual.updateStateMachine(DriveManualTrigger.SWITCH_MODES);
                  }));
      driveXbox.povUp().onTrue(new ResetFieldCentric(true));
      driveXbox.povDown().onTrue(driveStop);
      driveXbox
          .rightTrigger()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    RobotCoordinator.getInstance().setIntakeButtonState(true);
                    outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_COLLECTING_NOTE);
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
      operatorXbox.leftTrigger().onTrue(Commands.runOnce(()-> {
        RobotCoordinator.getInstance().setSlowClimbButtonHeld(true);
      }));
      operatorXbox.leftTrigger().onFalse(Commands.runOnce(()-> {
        RobotCoordinator.getInstance().setSlowClimbButtonHeld(false);
      }));
      operatorXbox
        .leftBumper()
        .whileTrue(new ClimberRetract());
      operatorXbox
        .rightBumper()
        .whileTrue(new ClimberExtend());
      operatorXbox.rightTrigger().whileTrue(new EjectThroughOuttake());
      operatorXbox.start().onTrue(new SetPivotsCoastMode());
      operatorXbox.back().onTrue(new SetPivotsBrakeMode());
      operatorXbox.povUp().whileTrue(new EjectThroughIntake());
      operatorXbox
          .y()
          .onTrue(
              Commands.runOnce(
                  () ->
                      outtakeManual.updateStateMachine(
                          OuttakeManualTrigger.ENABLE_SMART_SHOOTING)));
      operatorXbox
          .x()
          .onTrue(
              Commands.runOnce(
                  () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_EJECT)));
      operatorXbox
          .b()
          .onTrue(
              Commands.runOnce(
                  () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_SUBWOOFER)));
      operatorXbox
          .a()
          .onTrue(
              Commands.runOnce(
                  () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_STOP)));
    }
  }

  public void disabledPeriodic() {
    if (disableTimer.hasElapsed(Constants.DriveConstants.disableBreakSec)) {
      if (Constants.driveEnabled) {
        drive.setCoastMode(); // robot has stopped, safe to enter coast mode
      }
      disableTimer.stop();
      disableTimer.reset();
    }
  }

  public void teleopPeriodic() {
    // if robot crossing from our side to opponent side
    if (!robotCoordinator.onOurSideOfField() && !onOpponentFieldSide) {
      onOpponentFieldSide = true;
    }
    // if robot crossing from opponent side to our side
    else if (robotCoordinator.onOurSideOfField() && onOpponentFieldSide) {
      // removed so driver has to manually change it.
      /*Commands.runOnce(
      () -> {
        outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_SMART_SHOOTING);
      });*/
      onOpponentFieldSide = false;
    }
  }

  public void enableSubsystems() {
    drive.setBrakeMode();
    tunnel.setBrakeMode();
    intake.setIntakeBrakeMode();
    intake.setDeployerBrakeMode();
    outtake.setPivotBrakeMode();

    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    tunnel.setCoastMode();
    intake.setIntakeCoastMode();

    driveStop.schedule(); // interrupt all drive commands
    intakeStop.schedule(); // interrupt all intake commands
    outtakeStop.schedule(); // interrupt all outtake commands
    tunnelStop.schedule(); // interrupt all tunnel commands

    driveManual.updateStateMachine(DriveManualTrigger.RESET_TO_DEFAULT);

    disableTimer.reset();
    disableTimer.start();
  }

  // Command that should always start off every auto
  public Command getAutoInitialize() {
    final String autoName = AutoHelper.getPathPlannerAutoName(autoChooser.getSelected());
    if (autoName == "None") {
      return new SequentialCommandGroup(new ResetFieldCentric(true));
    } else {
      return new SequentialCommandGroup(
          new ResetFieldCentric(
              true,
              PathPlannerManager.getInstance()
                  .getStartingPoseFromAutoFile(autoName)
                  .getRotation()));
    }
  }

  // Command for the auto on our side of the field (PathPlanner Auto)
  public Command getAutoOurSide() {
    final String autoName = AutoHelper.getPathPlannerAutoName(autoChooser.getSelected());
    if (autoName == "None") {
      return Commands.none();
    } else {
      return PathPlannerManager.getInstance().buildAuto(autoName);
    }
  }

  public CenterLineScoringStrategy getCenterLineStrategy() {
    return AutoHelper.getCenterLineScoringStrategy(autoChooser.getSelected());
  }
}
