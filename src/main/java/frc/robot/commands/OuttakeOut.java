package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.PositionVector;

public class OuttakeOut extends Command {
  Outtake outtake;
  FiringSolutionManager firingSolutionManager;
  RobotCoordinator robotCoordinator;

  public OuttakeOut() {
    outtake = Outtake.getInstance();
    firingSolutionManager = FiringSolutionManager.getInstance();
    robotCoordinator = RobotCoordinator.getInstance();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (robotCoordinator.isAcrossCenterLine()) {
      outtake.outtake(
        firingSolutionManager.calcSolution(
          PositionVector.getMag(robotCoordinator.getRobotXPos(), robotCoordinator.getRobotYPos()),
          PositionVector.getAngle(robotCoordinator.getRobotXPos(), robotCoordinator.getRobotYPos()).getDegrees())
            .getFlywheelSpeed());
    }
  }

  @Override
  public boolean isFinished() {
    return !robotCoordinator.isAcrossCenterLine();
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
  }
}
