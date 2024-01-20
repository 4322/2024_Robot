package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;

public interface AutoChooserIO {
  @AutoLog
  public static class AutoChooserIOInputs {
    public int startingGridPosition;
    public Command autoCommand;
  }

  public default void updateInputs(AutoChooserIOInputs inputs) {}

  public default void loadAutos() {}
}
