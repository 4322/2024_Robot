package frc.robot.commands.DriveAuto;

public class NoteChecker {
  private final int amntOfCyclesToPass = 20;
  private int currentCycles = 0;

  public boolean CyclesMet() {
    if (NotePresentCycle()) {
      currentCycles++;
    } else {
      currentCycles = 0;
    }

    if (currentCycles >= amntOfCyclesToPass) {
      return true;
    }
    return false;
  }

  private boolean NotePresentCycle() {
    // TODO replaced with an overide in extended classes that is hardware dependent.
    return false;
  }
}
