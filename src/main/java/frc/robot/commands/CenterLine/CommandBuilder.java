package frc.robot.commands.CenterLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CenterLine.statemachine.CLSM.TravelState;

public class CommandBuilder {

  public static Command buildCommand(TravelState state) {
    switch (state) {
      default:
        return Commands.none();
    }
  }
}
