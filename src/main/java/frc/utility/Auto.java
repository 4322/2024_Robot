package frc.utility;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

public class Auto {
  public String name;
  public Command command;
  public List<Integer> positions;

  public Auto(String name, Command command, List<Integer> positions) {
    this.name = name;
    this.command = command;
    this.positions = positions;
  }
}
