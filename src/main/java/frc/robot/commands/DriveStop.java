package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveStop extends InstantCommand{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Used to interrupt all other drive commands and stop the drive

  private final Drive drive;

  public DriveStop(Drive drivesubsystem) {
    drive = drivesubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;  // allow drive to be stopped before re-enabling
  }
}
