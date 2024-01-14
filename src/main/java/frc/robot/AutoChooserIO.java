package frc.robot;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutoChooserIO {
  @AutoLog
  public static class AutoChooserIOInputs {
    public int startingGridPosition;
    public Command autoCommand;
  }

  public default void updateInputs(AutoChooserIOInputs inputs) {}

  public default void loadAutos() {}
}
