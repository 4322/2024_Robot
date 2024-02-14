package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
// may change this to take firing solution

public class OuttakeOut extends Command {
  Outtake outtake;
  FiringSolutionManager firingSolutionManager;
  Translation2d vectorToSpeaker;

  public OuttakeOut(Translation2d vector) {
    outtake = Outtake.getInstance();
    firingSolutionManager = FiringSolutionManager.getInstance();
    vectorToSpeaker = vector;

    addRequirements(outtake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().isAcrossCenterLine()) {
      outtake.outtake(
        firingSolutionManager.calcSolution(
          vectorToSpeaker.getDistance(vectorToSpeaker), vectorToSpeaker.getAngle().getDegrees())
            .getFlywheelSpeed());
    }
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
  }
}
