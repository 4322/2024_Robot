package frc.robot.commands.CenterLine.environmentTracker;

import frc.robot.commands.CenterLine.statemachine.CLSM.CLSMState;

public class EnvironmentTracker {
  private boolean note1Available;
  private boolean note2Available;
  private boolean note3Available;
  private boolean note4Available;
  private boolean note5Available;

  public EnvironmentTracker(NoteStatus status) {
    this.note1Available = status.note1Available;
    this.note2Available = status.note2Available;
    this.note3Available = status.note3Available;
    this.note4Available = status.note4Available;
    this.note5Available = status.note5Available;
  }

  public void update(CLSMState state) {
    switch (state) {
      case Note1:
        note1Available = false;
        break;
      case Note2:
        note2Available = false;
        break;
      case Note3:
        note3Available = false;
        break;
      case Note4:
        note4Available = false;
        break;
      case Note5:
        note5Available = false;
        break;
      default:
        return;
    }
  }

  public NoteStatus getNoteStatus() {
    return new NoteStatus(
        note1Available, note2Available, note3Available, note4Available, note5Available);
  }
}
