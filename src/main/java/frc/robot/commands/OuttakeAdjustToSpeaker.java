package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.FiringSolutionHelper;

public class OuttakeAdjustToSpeaker extends Command {
  Outtake outtake;

  public OuttakeAdjustToSpeaker() {
    outtake = Outtake.getInstance();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().onOurSideOfField()
        && RobotCoordinator.getInstance().canPivot()
        && outtake.safeToPivot()) {
      FiringSolution firingSolution =
          FiringSolutionManager.getInstance()
              .calcSolution(
                  FiringSolutionHelper.getMag(
                      RobotCoordinator.getInstance().getRobotXPos(),
                      RobotCoordinator.getInstance().getRobotYPos()),
                  FiringSolutionHelper.getAngle(
                          RobotCoordinator.getInstance().getRobotXPos(),
                          RobotCoordinator.getInstance().getRobotYPos())
                      .getDegrees());

      outtake.outtake(firingSolution.getFlywheelSpeed());
      // divide by 360 because pivot uses rotations instead of degrees
      outtake.pivot(firingSolution.getShotAngle() / 360);
    }
  }

  @Override
  public boolean isFinished() {
    return !RobotCoordinator.getInstance().onOurSideOfField();
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
    outtake.stopPivot();
  }
}
