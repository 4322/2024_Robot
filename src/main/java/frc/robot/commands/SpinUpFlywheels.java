package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.PositionVector;

public class SpinUpFlywheels extends Command {
  Outtake outtake;

  public SpinUpFlywheels() {
    outtake = Outtake.getInstance();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().onOurSideOfField()) {
      outtake.outtake(
          FiringSolutionManager.getInstance()
              .calcSolution(
                  PositionVector.getMag(
                      RobotCoordinator.getInstance().getRobotXPos(),
                      RobotCoordinator.getInstance().getRobotYPos()),
                  PositionVector.getAngle(
                          RobotCoordinator.getInstance().getRobotXPos(),
                          RobotCoordinator.getInstance().getRobotYPos())
                      .getDegrees())
              .getFlywheelSpeed());
    }
  }

  @Override
  public boolean isFinished() {
    return !RobotCoordinator.getInstance().onOurSideOfField();
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
  }
}
