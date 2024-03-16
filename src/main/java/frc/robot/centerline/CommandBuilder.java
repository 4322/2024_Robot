package frc.robot.centerline;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.PathPlannerManager;
import frc.robot.centerline.CLSM.TravelState;
import frc.robot.commands.SetOuttake;
import frc.robot.commands.Shoot;

public class CommandBuilder {
  public static Command buildCommand(TravelState state) {
    switch (state) {
      case N1ToN2:
      case N2ToN3:
      case N3ToN4:
      case N4ToN5:
      case N5ToN4:
      case N4ToN3:
      case N3ToN2:
      case N2ToN1:
        return PathPlannerManager.getInstance().followChoreoPath(state.toString());
      case TSToN1:
      case TSToN2:
      case TSToN3:
      case MSToN3:
      case MSToN4:
      case BSToN4:
      case BSToN5:
        return PathPlannerManager.getInstance().followChoreoPath(state.toString());
      case N1ToTS:
      case N2ToTS:
      case N3ToTS:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathPlannerManager.getInstance().followChoreoPath(state.toString()),
                new SetOuttake(Constants.FiringSolutions.TS)),
            new Shoot());
      case N3ToMS:
      case N4ToMS:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathPlannerManager.getInstance().followChoreoPath(state.toString()),
                new SetOuttake(Constants.FiringSolutions.MS)),
            new Shoot());
      case N4ToBS:
      case N5ToBS:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                PathPlannerManager.getInstance().followChoreoPath(state.toString()),
                new SetOuttake(Constants.FiringSolutions.BS)),
            new Shoot());
      case BSToBottomEndPos:
      case N5ToBottomEndPos:
        return PathPlannerManager.getInstance().followChoreoPath(state.toString());
      case None:
      case Done:
      default:
        return Commands.none();
    }
  }
}
