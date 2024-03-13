package frc.robot.commands;

import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;

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
    double botMagToSpeaker =
        FiringSolutionHelper.getVectorToSpeaker(
                RobotCoordinator.getInstance().getRobotXPos(),
                RobotCoordinator.getInstance().getRobotYPos())
            .getNorm();
    double botAngleToSpeaker =
        FiringSolutionHelper.getVectorToSpeaker(
                RobotCoordinator.getInstance().getRobotXPos(),
                RobotCoordinator.getInstance().getRobotYPos())
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
      outtake.pivot(solution.getShotRotations());
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
