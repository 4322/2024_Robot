package frc.robot.commands.CenterLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PathPlannerManager;
import frc.robot.commands.CenterLine.statemachine.CLSM.TravelState;

public class CommandBuilder {
  private final PathPlannerManager manager;

  public CommandBuilder(PathPlannerManager pathPlannerManager) {
    manager = pathPlannerManager;
  }

  public Command buildCommand(TravelState state) {
    switch (state) {
      case None:
      case Done:
        return Commands.none();
      default:
        return manager.getAuto(state.toString());
    }
  }
}
