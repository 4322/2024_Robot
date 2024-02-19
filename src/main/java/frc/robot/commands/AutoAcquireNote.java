package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.utility.OrangeMath;

public class AutoAcquireNote extends Command {

  private Drive drive;
  private RobotCoordinator coordinator;
  private Double initTx;
  private Double initTy;
  private boolean initialized;

  private double desiredHeadingAngle;
  private double desiredRobotDirectionX;
  private double desiredRobotDirectionY;
  private Double notePositionY; // field centric
  private Double notePositionX; // field centric

  public AutoAcquireNote() {
    drive = Drive.getInstance();
    coordinator = RobotCoordinator.getInstance();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    initTx = coordinator.getNearestNoteTX();
    initTy = coordinator.getNearestNoteTY();

    initialized = true;

    if (!coordinator.noteInVision()) {
      initialized =
          false; // this cancels auto aquisition (although the button will automatically restart it)
      return;
    }
    UpdateHeading();
  }

  private void UpdateHeading() {
    Double tx = coordinator.getNearestNoteTX();
    Double ty = coordinator.getNearestNoteTY();
    // I'm assuming that null from either implies that the data being returned is bad / donut is
    // lost on camera
    // this probably means that we are close to the donut and should use an approach speed.
    if (!coordinator.noteInVision()) {
      // dont update, we've lost the target. use old data.
      // TODO: we should mark this in the LED subsystem when that becomes a thing.
      return;
    }
    double yNoteDistance = Constants.LimelightConstants.intakeLimelightHeight * Math.tan(Math.toRadians(65 - ty));
    double xNoteDistance = Math.tan(Math.toRadians(tx)) * yNoteDistance;
    double noteDistance =
        Math.sqrt(xNoteDistance * xNoteDistance + yNoteDistance * yNoteDistance)
            + OrangeMath.inchesToMeters(
                Constants.noteRadiusInches); // get the distance to the far end of the donut
    Pose2d pose = drive.getPose2d();
    desiredHeadingAngle = pose.getRotation().getRadians() - Math.atan2(xNoteDistance, yNoteDistance);
    desiredRobotDirectionX = Math.cos(desiredHeadingAngle);
    desiredRobotDirectionY = Math.sin(desiredHeadingAngle);
    notePositionX = noteDistance * desiredRobotDirectionX + pose.getX();
    notePositionY = noteDistance * desiredRobotDirectionY + pose.getY();
      
  }

  @Override
  public void execute() {
    double approachSpeed =
        1; // set to something that is more intricate if we need to slow down on approach.
    Pose2d pose = drive.getPose2d();
    // drive if we aren't too close to the goal spot
    if (distance(pose.getX(), pose.getY(), notePositionX, notePositionY)
        > OrangeMath.inchesToMeters(Constants.noteRadiusInches)) {
      drive.driveAutoRotate(
          approachSpeed * desiredRobotDirectionX,
          approachSpeed * desiredRobotDirectionY,
          desiredHeadingAngle);
    }
    UpdateHeading();
  }

  @Override
  public boolean isFinished() {
    return !initialized
        || coordinator.noteInRobot()
        || !coordinator.getAutoIntakeButtonPressed()
        || Intake.getInstance().getState() != Intake.IntakeStates.feeding
        || !coordinator.noteInVision();
  }

  @Override
  public void end(boolean interrupted) {
    initialized = false;
  }

  private static double distance(double x1, double y1, double x2, double y2) {
    return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }
}
