package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveInterface;

public class SetRobotPose extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveInterface driveSubsystem;

  private final Pose2d pose;
  private final boolean runWhenEnabled;

  public SetRobotPose(DriveInterface drive, Pose2d pose, boolean runWhenEnabled) {
    this.driveSubsystem = drive;
    this.pose = pose;
    this.runWhenEnabled = runWhenEnabled;

    // Interrupt the active DriveManual command just in case we use the pose in there
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDisabled() || runWhenEnabled) {
      driveSubsystem.resetOdometry(pose);
      // TODO: AdvantageKit
      String report = "Reset Field Pose";
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
