package frc.robot.commands.CenterLine.environmentTracker;

public class NoteStatus {
  public final boolean note1Available;
  public final boolean note2Available;
  public final boolean note3Available;
  public final boolean note4Available;
  public final boolean note5Available;

  public NoteStatus(
      boolean note1Available,
      boolean note2Available,
      boolean note3Available,
      boolean note4Available,
      boolean note5Available) {
    this.note1Available = note1Available;
    this.note2Available = note2Available;
    this.note3Available = note3Available;
    this.note4Available = note4Available;
    this.note5Available = note5Available;
  }

  @Override
  public String toString() {
    String str = "";
    str += (note1Available) ? "1" : "0";
    str += (note2Available) ? "1" : "0";
    str += (note3Available) ? "1" : "0";
    str += (note4Available) ? "1" : "0";
    str += (note5Available) ? "1" : "0";
    return str;
  }

  @Override
  public boolean equals(Object obj) {
    boolean result = false;
    if (obj instanceof NoteStatus) {
      NoteStatus other = (NoteStatus) obj;
      result =
          (note1Available == other.note1Available
              && note2Available == other.note2Available
              && note3Available == other.note3Available
              && note4Available == other.note4Available
              && note5Available == other.note5Available);
    }
    return result;
  }
}
