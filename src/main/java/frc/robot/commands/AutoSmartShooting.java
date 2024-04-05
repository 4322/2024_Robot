package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;
import org.littletonrobotics.junction.Logger;

public class AutoSmartShooting extends InstantCommand {
  private final Outtake outtake;

  public AutoSmartShooting() {
    outtake = Outtake.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    final FiringSolution solution;
    final Pose2d botpose = Limelight.getOuttakeInstance().getBotposeWpiBlue();
    double botMagToSpeaker =
        FiringSolutionHelper.getVectorToSpeaker(
                botpose.getX(),
                botpose.getY())
            .getNorm();
    double botAngleToSpeaker =
        FiringSolutionHelper.getVectorToSpeaker(
                botpose.getX(),
                botpose.getY())
            .getAngle()
            .getDegrees();
    solution = FiringSolutionManager.getInstance().calcSolution(botMagToSpeaker, botAngleToSpeaker);

    Logger.recordOutput("FiringSolutions/CalculatedShot", solution.toString());
    Logger.recordOutput("FiringSolutions/BotPoseInput/Mag", botMagToSpeaker);
    Logger.recordOutput("FiringSolutions/BotPoseInput/Angle", botAngleToSpeaker);

    if (RobotCoordinator.getInstance().canSpinFlywheel()) {
      outtake.outtake(solution.getFlywheelSpeed());
    }

    if (RobotCoordinator.getInstance().canPivot()) {
      // 3.5 rotation addition to adjust for error at AVR
      outtake.pivot(solution.getShotRotations() + 2.5);
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
