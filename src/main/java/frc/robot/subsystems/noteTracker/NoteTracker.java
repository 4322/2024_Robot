package frc.robot.subsystems.noteTracker;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BeamBreakConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;

import org.littletonrobotics.junction.Logger;

public class NoteTracker extends SubsystemBase {
  private static NoteTrackerIO noteTrackerIO;
  private static NoteTrackerIOInputsAutoLogged inputs = new NoteTrackerIOInputsAutoLogged();

  private boolean notePassingIntake;
  private boolean notePassingTunnel;
  private boolean noteIsShot;
  private Timer shootTimer = new Timer();

  private static NoteTracker noteTracker;

  public static NoteTracker getInstance() {
    if (noteTracker == null) {
      noteTracker = new NoteTracker();
    }
    return noteTracker;
  }

  public NoteTracker() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.sensorsEnabled) {
          noteTrackerIO = new NoteTrackerIOReal();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (noteTrackerIO == null) {
      noteTrackerIO = new NoteTrackerIO() {};
    }
  }

  @Override
  public void periodic() {
    if (Constants.sensorsEnabled) {
      noteTrackerIO.updateInputs(inputs);
      Logger.processInputs(BeamBreakConstants.Logging.key, inputs);

      // update note tracking logic in robot
      if (!inputs.intakeBeamBreak && Intake.getInstance().isFeeding()) {
        notePassingIntake = true;
      } else if (!inputs.tunnelBeamBreak) {
        notePassingIntake = false;
        notePassingTunnel = true;
      } else if (inputs.tunnelBeamBreak && notePassingTunnel && Outtake.getInstance().isOuttaking()) {
        shootTimer.start();
        noteIsShot = true;
        if (shootTimer.hasElapsed(0.2)) {
          notePassingTunnel = false;
          shootTimer.stop();
          shootTimer.reset();
          noteIsShot = false;
        }
      } else if (Intake.getInstance().isEjecting()) {
        notePassingIntake = false;
      }
    }
  }

  public boolean intakeBeamBroken() {
    return !inputs.intakeBeamBreak;
  }

  public boolean tunnelBeamBroken() {
    return !inputs.tunnelBeamBreak;
  }

  public boolean notePassingIntake() {
    return notePassingIntake;
  }

  public boolean notePassingTunnel() {
    return notePassingTunnel;
  }

  public boolean noteIsShot() {
    return noteIsShot;
  }
}
