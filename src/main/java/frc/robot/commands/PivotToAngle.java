package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtakePivot.OuttakePivot;
import frc.utility.PositionVector;

public class PivotToAngle extends Command {
  OuttakePivot outtakePivot;
  RobotCoordinator robotCoordinator;
  FiringSolutionManager firingSolutionManager;

  public PivotToAngle() {
    outtakePivot = OuttakePivot.getInstance();
    robotCoordinator = RobotCoordinator.getInstance();
    firingSolutionManager = FiringSolutionManager.getInstance();
    addRequirements(outtakePivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (robotCoordinator.canPivot() && robotCoordinator.onOurSideOfField()) {
      // divide by 360 because pivot uses rotations instead of degrees
      outtakePivot.pivot(
          firingSolutionManager
                  .calcSolution(
                      PositionVector.getMag(
                          robotCoordinator.getRobotXPos(), robotCoordinator.getRobotYPos()),
                      PositionVector.getAngle(
                              robotCoordinator.getRobotXPos(), robotCoordinator.getRobotYPos())
                          .getDegrees())
                  .getShotAngle()
              / 360);
    }
  }

  @Override
  public boolean isFinished() {
    return !robotCoordinator.onOurSideOfField();
  }

  @Override
  public void end(boolean interrupted) {
    outtakePivot.stopPivot();
  }
}
