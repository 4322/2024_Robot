package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.Limelight;

public class UpdateOdometry extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Limelight limelight = Limelight.getOuttakeInstance();

  public UpdateOdometry() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    final Pose2d limelightPose = limelight.getBotposeWpiBlue();
    final double distanceToBot =
        Drive.getInstance()
            .getPose2d()
            .getTranslation()
            .getDistance(limelightPose.getTranslation());
    final boolean withinAcceptableDistance =
        distanceToBot <= LimelightConstants.visionOdometryTolerance;
    final boolean pastVisionOverrideTolerance =
        distanceToBot >= LimelightConstants.reverseOdometryOverrideTolerance;
    final int numTargets = limelight.getNumTargets();
    if (limelight.getTargetVisible()
        && numTargets >= LimelightConstants.numTargetsToUseReverseOdom) {
      if (withinAcceptableDistance) {
        Drive.getInstance()
            .updateOdometryVision(
                limelightPose, Timer.getFPGATimestamp() - limelight.getTotalLatency());
      } else if (pastVisionOverrideTolerance) {
        Drive.getInstance().resetOdometry(limelightPose);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
