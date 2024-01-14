package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ResetFieldCentric extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drive driveSubsystem;
  private final double offset;
  private final boolean runWhenEnabled;

  public ResetFieldCentric(Drive driveSubsystem, double offset, boolean runWhenEnabled) {
    this.driveSubsystem = driveSubsystem;
    this.offset = offset;
    this.runWhenEnabled = runWhenEnabled;
    // Interrupt the active DriveManual command so we don't auto-rotate
    // back to the old heading lock that is no longer valid after the reset.
    addRequirements(driveSubsystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDisabled() || runWhenEnabled) {
      driveSubsystem.resetFieldCentric(offset);
      String report = "Reset Field Centric for " + offset + " degrees";
      DriverStation.reportWarning(report, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;  // allow field orientation to be set before start of match
  }
}
