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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AutoHelper.Auto;
import frc.robot.Constants.ControllerRumbleTimes;
import frc.robot.centerline.CenterLineManager.CenterLineScoringStrategy;
import frc.robot.commands.AutoIntakeDeploy;
import frc.robot.commands.AutoIntakeIn;
import frc.robot.commands.AutoSetOuttakeAdjust;
import frc.robot.commands.AutoSmartShooting;
import frc.robot.commands.DriveManual.DriveManual;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualTrigger;
import frc.robot.commands.DriveStop;
import frc.robot.commands.EjectThroughIntake;
import frc.robot.commands.IntakeManual;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.LEDState;
import frc.robot.commands.OperatorPresetLED;
import frc.robot.commands.OperatorXboxControllerRumble;
import frc.robot.commands.OuttakeManual.OuttakeManual;
import frc.robot.commands.OuttakeManual.OuttakeManualStateMachine.OuttakeManualTrigger;
import frc.robot.commands.OuttakeStop;
import frc.robot.commands.OuttakeTunnelFeed.OuttakeTunnelFeed;
import frc.robot.commands.ResetFieldCentric;
import frc.robot.commands.SetPivotsBrakeMode;
import frc.robot.commands.SetPivotsCoastMode;
import frc.robot.commands.SetRobotPose;
import frc.robot.commands.Shoot;
import frc.robot.commands.TunnelFeed;
import frc.robot.commands.TunnelStop;
import frc.robot.commands.UpdateOdometry;
import frc.robot.commands.WriteFiringSolutionAtCurrentPos;
import frc.robot.commands.DriverXboxControllerRumble;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.limelight.Limelight;
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
  private final LED led = LED.getInstance();

  private final WriteFiringSolutionAtCurrentPos writeFiringSolution =
      new WriteFiringSolutionAtCurrentPos();

  private final DriveManual driveManual = new DriveManual();

  private final TunnelFeed tunnelFeedContinuous = new TunnelFeed(false);

  private final OuttakeManual outtakeManual = new OuttakeManual();

  private final IntakeManual intakeManual = new IntakeManual();

  private final DriveStop driveStop = new DriveStop();

  private final IntakeStop intakeStop = new IntakeStop();

  private final OuttakeStop outtakeStop = new OuttakeStop();

  private final TunnelStop tunnelStop = new TunnelStop();

  private final LEDState defaultLEDStates = new LEDState();

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
        .addEvent(
            "SetOuttakeSubwooferSide",
            new AutoSetOuttakeAdjust(Constants.FiringSolutions.SubwooferSide));
    PathPlannerManager.getInstance()
        .addEvent(
            "SetOuttakeCollectingNote",
            new AutoSetOuttakeAdjust(Constants.FiringSolutions.CollectingNote));
    PathPlannerManager.getInstance().addEvent("SetOuttakeSmartShooting", new AutoSmartShooting());
    PathPlannerManager.getInstance().addEvent("TunnelFeed", new TunnelFeed(true));

    PathPlannerManager.getInstance()
        .addEvent(
            "SetOuttakeN6",
            new AutoSetOuttakeAdjust(new FiringSolution(0, 0, 40, 40)));
    PathPlannerManager.getInstance()
        .addEvent(
            "SetOuttakeN7",
            new AutoSetOuttakeAdjust(new FiringSolution(0, 0, 40, 40)));
    PathPlannerManager.getInstance()
        .addEvent(
            "SetOuttakeN8",
            new AutoSetOuttakeAdjust(new FiringSolution(0, 0, 40, 40)));

    // DO NOT MOVE OR REMOVE THIS WITHOUT KNOWING WHAT YOU'RE DOING
    PathPlannerManager.getInstance().preloadAutos();

    autoChooser = new SendableChooser<>();
    AutoHelper.configAutoChooser(autoChooser);
    Shuffleboard.getTab("Autos").add(autoChooser).withPosition(0, 0).withSize(5, 2);

    FiringSolutionManager.getInstance().loadSolutions();

    // Records branch name and commit hash to only Driver station log (doesn't output to console)
    System.out.println("Git branch in use: " + BuildConstants.GIT_BRANCH);
    System.out.println("Git commit hash in use: " + BuildConstants.GIT_SHA);

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManual);
    }

    if (Constants.tunnelEnabled) {
      tunnel.setDefaultCommand(tunnelFeedContinuous);
    }

    if (Constants.outtakeEnabled || Constants.outtakePivotEnabled) {
      outtake.setDefaultCommand(outtakeManual);
    }

    if (Constants.intakeEnabled || Constants.intakeDeployerEnabled) {
      intake.setDefaultCommand(intakeManual);
    }

    if (Constants.outtakeLimeLightEnabled) {
      Limelight.getOuttakeInstance().setDefaultCommand(new UpdateOdometry());
    }

    if (Constants.ledEnabled) {
      led.setDefaultCommand(defaultLEDStates);
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
      if (Constants.speakerCentricEnabled) {
        driveXbox
            .start() // binded to back right P2 button on xbox
            .onTrue(
                Commands.runOnce(
                    () -> {
                      driveManual.updateStateMachine(DriveManualTrigger.ENABLE_SPEAKER_CENTRIC);
                    }));
        driveXbox
            .start()
            .onFalse(
                Commands.runOnce(
                    () -> {
                      driveManual.updateStateMachine(DriveManualTrigger.RESET_TO_DEFAULT);
                    }));
      }
      driveXbox
            .back() // binded to back left P2 button on xbox
            .onTrue(
                Commands.runOnce(
                    () -> {
                      driveManual.updateStateMachine(DriveManualTrigger.ENABLE_PASS);
                    }));
        driveXbox
            .back()
            .onFalse(
                Commands.runOnce(
                    () -> {
                      driveManual.updateStateMachine(DriveManualTrigger.RESET_TO_DEFAULT);
                    }));
      driveXbox.x().onTrue(new ResetFieldCentric(true));
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
                    outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_STOP);
                  }));
      driveXbox.leftTrigger().whileTrue(new Shoot());
      driveXbox.rightBumper().onTrue(Commands.runOnce(() -> {driveManual.updateStateMachine(DriveManualTrigger.ENABLE_AMP);}));
      driveXbox.rightBumper().onFalse(Commands.runOnce(() -> {driveManual.updateStateMachine(DriveManualTrigger.RESET_TO_DEFAULT);}));
      driveXbox.leftBumper().onTrue(Commands.runOnce(() -> {driveManual.updateStateMachine(DriveManualTrigger.ENABLE_SOURCE);}));
      driveXbox.leftBumper().onFalse(Commands.runOnce(() -> {driveManual.updateStateMachine(DriveManualTrigger.RESET_TO_DEFAULT);}));
      if (Constants.outtakeTuningMode) {
        driveXbox.y().onTrue(writeFiringSolution);
        // right up against front of speaker with edge of robot on source side
        driveXbox
            .a()
            .onTrue(
                new SetRobotPose(
                    new Pose2d(1.3766260147094727, 5.414320468902588, new Rotation2d()), true));
      }
      operatorXbox.start().onTrue(new SetPivotsCoastMode());
      operatorXbox.back().onTrue(new SetPivotsBrakeMode());
      operatorXbox.povUp().whileTrue(new EjectThroughIntake());
      operatorXbox
          .y()
          .onTrue(new ParallelCommandGroup(
              Commands.runOnce(() -> {
                outtakeManual.setFiringSolution(Constants.FiringSolutions.DefaultSmartShooting);
                outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_SMART_SHOOTING);}),
                new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime)));
      operatorXbox
          .x()
          .onTrue(new ParallelCommandGroup(
              Commands.runOnce(
                  () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_EJECT)),
                  new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime)));
      operatorXbox
          .b()
          .onTrue(new ParallelCommandGroup(
              Commands.runOnce(
                  () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_SUBWOOFER)),
                  new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime)));
      operatorXbox
          .a()
          .onTrue(new ParallelCommandGroup(
              Commands.runOnce(
                  () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_STOP)),
                  new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime)));
      operatorXbox
          .povDown()
          .onTrue(
              new SequentialCommandGroup(
                  Commands.runOnce(
                      () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_FEED)),
                  new OuttakeTunnelFeed(),
                  new OperatorXboxControllerRumble(ControllerRumbleTimes.longRumbleTime)));
      operatorXbox.povDown().onTrue(new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime));
      operatorXbox
          .povRight()
          .onTrue(new ParallelCommandGroup(
              Commands.runOnce(
                  () -> outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_AMP)),
                  new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime)));
      operatorXbox.povLeft().onTrue(Commands.runOnce(() -> {outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_STARTING_CONFIG);}));
      operatorXbox.leftBumper().onTrue(new SequentialCommandGroup(Commands.runOnce(() -> {outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_WING_LINE_PASS);}), 
                  new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime)));
      operatorXbox.rightBumper().onTrue(new SequentialCommandGroup(Commands.runOnce(() -> {outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_FLAT_PASS);}),
                  new OperatorXboxControllerRumble(ControllerRumbleTimes.shortRumbleTime)));
      operatorXbox.leftTrigger().onTrue(Commands.runOnce(() -> {outtakeManual.addOffset(0.5);}));
      operatorXbox.rightTrigger().onTrue(Commands.runOnce(() -> {outtakeManual.addOffset(-0.5);}));

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
    intake.setDeployerBrakeMode();
    outtake.setPivotBrakeMode();

    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    tunnel.setCoastMode();
    outtake.setFlywheelCoastMode();

    driveStop.schedule(); // interrupt all drive commands
    intakeStop.schedule(); // interrupt all intake commands
    outtakeStop.schedule(); // interrupt all outtake commands
    tunnelStop.schedule(); // interrupt all tunnel commands

    driveManual.updateStateMachine(DriveManualTrigger.RESET_TO_DEFAULT);
    outtakeManual.updateStateMachine(OuttakeManualTrigger.ENABLE_STOP);

    disableTimer.reset();
    disableTimer.start();
  }

  // Command that should always start off every auto
  public Command getAutoInitialize() {
    // final String autoName = AutoHelper.getPathPlannerAutoName(autoChooser.getSelected());
    // if (autoName == "None") {
    //   return new SequentialCommandGroup(new ResetFieldCentric(true));
    // } else {
    //   return new SequentialCommandGroup(
    //       new ResetFieldCentric(
    //           true,
    //           PathPlannerManager.getInstance()
    //               .getStartingPoseFromAutoFile(autoName)
    //               .getRotation()));
    // }
    return Commands.none();
  }

  // Command for the auto on our side of the field (PathPlanner Auto)
  public Command getAutoOurSide() {
    final String autoName = AutoHelper.getPathPlannerAutoName(autoChooser.getSelected());
    if (autoName == "None") {
      return Commands.none();
    } else {
      return PathPlannerManager.getInstance().getAuto(autoName);
    }
  }

  public CenterLineScoringStrategy getCenterLineStrategy() {
    return AutoHelper.getCenterLineScoringStrategy(autoChooser.getSelected());
  }
}
