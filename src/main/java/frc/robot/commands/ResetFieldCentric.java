package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

public class ResetFieldCentric extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive driveSubsystem;

  private final boolean runWhenEnabled;

  public ResetFieldCentric(Drive driveSubsystem, boolean runWhenEnabled) {
    this.driveSubsystem = driveSubsystem;
    this.runWhenEnabled = runWhenEnabled;
    // Interrupt the active DriveManual command so we don't auto-rotate
    // back to the old heading lock that is no longer valid after the reset.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDisabled() || runWhenEnabled) {
      driveSubsystem.resetFieldCentric();
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
