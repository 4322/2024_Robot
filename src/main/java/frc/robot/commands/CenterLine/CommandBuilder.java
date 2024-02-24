package frc.robot.commands.CenterLine;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PathPlannerManager;
import frc.robot.commands.CenterLine.statemachine.CLSM.TravelState;
import java.util.EnumMap;

public class CommandBuilder {
  EnumMap<TravelState, Command> pathCommands;

  public CommandBuilder() {
    pathCommands = new EnumMap<TravelState, Command>(TravelState.class);
    for (TravelState state : TravelState.values()) {
      if (!(state == TravelState.None || state == TravelState.Done)) {
        pathCommands.put(
            state,
            PathPlannerManager.getInstance()
                .followPath(PathPlannerPath.fromChoreoTrajectory(state.toString())));
      } else {
        pathCommands.put(state, Commands.none());
      }
    }
  }

  public Command buildCommand(TravelState state) {
    switch (state) {
      default:
        return pathCommands.get(state);
    }
  }
}
