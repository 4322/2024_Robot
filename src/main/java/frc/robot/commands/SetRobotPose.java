package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

public class SetRobotPose extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;

  private final Pose2d pose;
  private final boolean runWhenEnabled;

  public SetRobotPose(Pose2d pose, boolean runWhenEnabled) {
    drive = Drive.getInstance();
    this.pose = pose;
    this.runWhenEnabled = runWhenEnabled;

    // Interrupt the active DriveManual command just in case we use the pose in there
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.isDisabled() || runWhenEnabled) {
      drive.resetOdometry(pose);
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
