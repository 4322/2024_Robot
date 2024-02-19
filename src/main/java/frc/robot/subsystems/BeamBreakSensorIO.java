package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakSensorIO {
  @AutoLog
  public static class BeamBreakSensorIOInputs {
    public boolean tunnelBeamBreak = true;
    public boolean intakeBeamBreak = true;
  }

  public default void updateInputs(BeamBreakSensorIOInputs inputs) {}
}
