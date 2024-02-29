package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

public class ResetFieldCentric extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;

  private final boolean runWhenEnabled;
  private final Rotation2d robotRotation;

  public ResetFieldCentric(boolean runWhenEnabled) {
    drive = Drive.getInstance();
    this.runWhenEnabled = runWhenEnabled;
    this.robotRotation = new Rotation2d();
    // Interrupt the active DriveManual command so we don't auto-rotate
    // back to the old heading lock that is no longer valid after the reset.
    addRequirements(drive);
  }

  public ResetFieldCentric(boolean runWhenEnabled, Rotation2d robotRotation) {
    drive = Drive.getInstance();
    this.runWhenEnabled = runWhenEnabled;
    this.robotRotation = robotRotation;
    // Interrupt the active DriveManual command so we don't auto-rotate
    // back to the old heading lock that is no longer valid after the reset.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDisabled() || runWhenEnabled) {
      drive.resetFieldCentric(robotRotation.getDegrees());
      String report = "Reset Field Centric";
      DriverStation.reportWarning(report, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true; // allow field orientation to be set before start of match
  }
}
