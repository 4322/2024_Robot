package frc.robot.centerline;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PathPlannerManager;
import frc.robot.centerline.CLSM.TravelState;

public class CommandBuilder {
  public static Command buildCommand(TravelState state) {
    switch (state) {
      case None:
      case Done:
        return Commands.none();
      default:
        return PathPlannerManager.getInstance().followChoreoPath(state.toString());
    }
  }
}
