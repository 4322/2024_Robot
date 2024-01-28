package frc.robot.commands.DriveAuto;

public class NoteTracker {
  private final NoteChecker noteChecker;
  private boolean cyclesMet = false;

  public NoteTracker() {
    // TODO replace with a hardware extended version like "Limelight note checker"
    noteChecker = new NoteChecker() {};
  }

  public boolean isNoteInRightPos() {
    // TODO hardware implementation
    return false;
  }

  public boolean isHasGamePiece() {
    if (cyclesMet == true) {
      return true;
    }

    cyclesMet = noteChecker.CyclesMet();
    return cyclesMet;
  }
}
