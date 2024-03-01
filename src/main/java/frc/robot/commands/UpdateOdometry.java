package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.Limelight;

public class UpdateOdometry extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Timer updateTimer;

  public UpdateOdometry() {
    updateTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateTimer.reset();
    updateTimer.start();
  }

  @Override
  public void execute() {
    Pose2d limelightPose = Limelight.getOuttakeInstance().getBotposeWpiBlue();
    if (Limelight.getOuttakeInstance().getTargetVisible()
        && updateTimer.hasElapsed(LimelightConstants.odometryUpdatePeriodSeconds)
        && Drive.getInstance()
                .getPose2d()
                .getTranslation()
                .getDistance(limelightPose.getTranslation())
            < LimelightConstants.visionOdometryTolerance) {
      Drive.getInstance()
          .updateOdometryVision(
              limelightPose,
              Timer.getFPGATimestamp() - Limelight.getOuttakeInstance().getTotalLatency());
      updateTimer.reset();
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
