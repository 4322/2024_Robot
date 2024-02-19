package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED.LEDState;
import frc.robot.subsystems.outtake.Outtake;
import frc.utility.PositionVector;

public class OuttakeOut extends Command {
  Outtake outtake;

  public OuttakeOut() {
    outtake = Outtake.getInstance();

    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotCoordinator.getInstance().isAcrossCenterLine()) {
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
    return !RobotCoordinator.getInstance().isAcrossCenterLine();
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stopOuttake();
    LED.getInstance().setLEDState(LEDState.blue);
  }
}
