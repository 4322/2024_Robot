package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.drive.DriveInterface;
import frc.robot.subsystems.outtake.OuttakeInterface;
import frc.robot.subsystems.outtakePivot.OuttakePivotInterface;
import frc.utility.PositionVector;

public class WriteFiringSolutionAtCurrentPos extends InstantCommand {

  private FiringSolutionManager firingSolutionManager;
  private OuttakeInterface outtake;
  private DriveInterface drive;
  private OuttakePivotInterface outtakePivot;
  double shotAngle;
  double shotMag;

  public WriteFiringSolutionAtCurrentPos(
      OuttakeInterface outtakeSubsystem,
      DriveInterface drivesubsystem,
      OuttakePivotInterface pivot,
      FiringSolutionManager solutionManager) {
    outtake = outtakeSubsystem;
    drive = drivesubsystem;
    outtakePivot = pivot;
    firingSolutionManager = solutionManager;
  }

  @Override
  public void initialize() {
    Translation2d rawTranslation =
        PositionVector.getVectorToSpeaker(drive.getPose2d().getX(), drive.getPose2d().getY());
    shotAngle = rawTranslation.getAngle().getDegrees();
    // Calculates magnitude from x and y vals
    shotMag =
        Math.sqrt(
            rawTranslation.getX() * rawTranslation.getX()
                + rawTranslation.getY() * rawTranslation.getY());
    FiringSolution solution =
        new FiringSolution(shotMag, shotAngle, outtake.getTargetRPM(), outtakePivot.getTarget());
    firingSolutionManager.writeSolution(solution);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}
