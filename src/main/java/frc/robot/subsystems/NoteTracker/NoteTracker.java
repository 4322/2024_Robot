package frc.robot.subsystems.NoteTracker;

public class NoteTracker {
    private static NoteTracker noteTracker;

    public static NoteTracker getInstance() {
        if (noteTracker == null) {
            noteTracker = new NoteTracker();
        }
        return noteTracker;
    }

    public boolean hasNote() {
        return false;
    }

    public boolean isInPosition() {
        return false;
    }
}
