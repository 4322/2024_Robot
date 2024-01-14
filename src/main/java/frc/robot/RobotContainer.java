// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveManual;
import frc.robot.commands.DriveStop;
import frc.robot.commands.ResetFieldCentric;
import frc.robot.subsystems.drive.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
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

  private final Drive drive = Drive.getInstance(); 

  private final DriveManual driveManualDefault = new DriveManual(drive, DriveManual.AutoPose.none);
  private final DriveStop driveStop = new DriveStop(drive);

  private AutoChooserIO autoChooserIO;
  private AutoChooserIOInputsAutoLogged autoChooserInputs = new AutoChooserIOInputsAutoLogged();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        autoChooserIO = new AutoChooserIODataEntry(drive);
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
      drive.setDefaultCommand(driveManualDefault);
    }

    autoChooserIO = new AutoChooserIO() {};
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);
      
      driveButtonThree = new JoystickButton(driveStick, 3);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonTwelve = new JoystickButton(driveStick, 12);

      driveButtonThree.onTrue(new DriveManual(drive, DriveManual.AutoPose.usePresetAuto));
      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
      driveButtonTwelve.onTrue(driveStop);
    }

    if (Constants.xboxEnabled) {
      xbox = new CommandXboxController(2);
      xbox.povUp().onTrue(new ResetFieldCentric(drive, 0, true));
      xbox.rightBumper().onTrue(new DriveManual(drive, DriveManual.AutoPose.usePresetAuto));
      xbox.povDown().onTrue(driveStop);
    }
  }

  public void disabledPeriodic() {
    // update logs
    autoChooserIO.updateInputs(autoChooserInputs);
    Logger.getInstance().processInputs("AutoChooser", autoChooserInputs);

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
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    driveStop.schedule();  // interrupt all drive commands
    disableTimer.reset();
    disableTimer.start();
    
    // autos need to be reloaded after each auto test because the commands can't be reused
    autoChooserIO.loadAutos(); 
  }

  public Command getAutonomousCommand() {
    if (Constants.Demo.inDemoMode) {
      return null;
    }

    Logger.getInstance().recordOutput("Auto", autoChooserInputs.autoCommand.getName());
    
    return new SequentialCommandGroup(
      getAutoInitialize(),
      autoChooserInputs.autoCommand
    );
  }

  //AUTO COMMANDS

  // Command that should always start off every auto
  public Command getAutoInitialize() {
    return new SequentialCommandGroup(
      new ResetFieldCentric(drive, 0, true)
    );
  }
}
