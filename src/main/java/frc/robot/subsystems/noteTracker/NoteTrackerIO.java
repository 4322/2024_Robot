package frc.robot.subsystems.noteTracker;

import org.littletonrobotics.junction.AutoLog;

public interface NoteTrackerIO {
  @AutoLog
  public static class NoteTrackerIOInputs {
    public boolean tunnelBeamBreak = true;
    public boolean intakeBeamBreak = true;
  }

  public default void updateInputs(NoteTrackerIOInputs inputs) {}
}
