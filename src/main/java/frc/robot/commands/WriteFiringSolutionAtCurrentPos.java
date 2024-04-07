package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;

public class WriteFiringSolutionAtCurrentPos extends InstantCommand {

  private FiringSolutionManager firingSolutionManager;
  private Outtake outtake;
  double shotAngle;
  double shotMag;

  public WriteFiringSolutionAtCurrentPos() {
    outtake = Outtake.getInstance();
    firingSolutionManager = FiringSolutionManager.getInstance();
  }

  @Override
  public void initialize() {
    if (Constants.outtakeTuningMode) {
      if (Limelight.getOuttakeInstance().getTargetVisible()) {
        Translation2d rawTranslation =
          FiringSolutionHelper.getVectorToSpeaker(
              Limelight.getOuttakeInstance().getBotposeWpiBlue().getX(), 
                Limelight.getOuttakeInstance().getBotposeWpiBlue().getY());
        shotAngle = rawTranslation.getAngle().getDegrees();
        // Calculates magnitude from x and y vals
        shotMag = rawTranslation.getNorm();
        FiringSolution solution =
            // Doesn't matter if you log top or bottom target RPS to JSON.
            // Both shooter speeds will be the same value if we are doing shot tuning for speaker
            new FiringSolution(shotMag, shotAngle, outtake.getTopTargetRPS(), outtake.getPivotTarget());
        firingSolutionManager.writeSolution(solution);
      }
      
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}
