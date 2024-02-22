package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;

public class WriteFiringSolutionAtCurrentPos extends InstantCommand {

  private FiringSolutionManager firingSolutionManager;
  private Drive drive;
  private Outtake outtake;
  double shotAngle;
  double shotMag;

  public WriteFiringSolutionAtCurrentPos() {
    drive = Drive.getInstance();
    outtake = Outtake.getInstance();
    firingSolutionManager = FiringSolutionManager.getInstance();
  }

  @Override
  public void initialize() {
    if (Constants.inShotTuning) {
      Translation2d rawTranslation =
          FiringSolutionHelper.getVectorToSpeaker(
              drive.getPose2d().getX(), drive.getPose2d().getY());
      shotAngle = rawTranslation.getAngle().getDegrees();
      // Calculates magnitude from x and y vals
      shotMag =
          Math.sqrt(
              rawTranslation.getX() * rawTranslation.getX()
                  + rawTranslation.getY() * rawTranslation.getY());
      FiringSolution solution =
          new FiringSolution(shotMag, shotAngle, outtake.getTargetRPS(), outtake.getPivotTarget());
      firingSolutionManager.writeSolution(solution);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}
