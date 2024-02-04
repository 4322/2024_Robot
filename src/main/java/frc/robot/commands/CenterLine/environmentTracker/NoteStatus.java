package frc.robot.commands.CenterLine.environmentTracker;

public class NoteStatus {
  public final boolean note1Available;
  public final boolean note2Available;
  public final boolean note3Available;
  public final boolean note4Available;
  public final boolean note5Available;

  public NoteStatus(boolean note1Available, boolean note2Available, boolean note3Available, boolean note4Available,
      boolean note5Available) {
    this.note1Available = note1Available;
    this.note2Available = note2Available;
    this.note3Available = note3Available;
    this.note4Available = note4Available;
    this.note5Available = note5Available;
  }
}
