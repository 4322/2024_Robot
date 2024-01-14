package frc.utility;

import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;

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
