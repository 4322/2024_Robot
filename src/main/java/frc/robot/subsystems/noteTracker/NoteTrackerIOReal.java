package frc.robot.subsystems.noteTracker;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class NoteTrackerIOReal implements NoteTrackerIO {
  DigitalInput intakeInput;
  DigitalInput tunneInput;

  public NoteTrackerIOReal() {
    if (Constants.sensorsEnabled) {
      intakeInput = new DigitalInput(Constants.BeamBreakConstants.intakeBeamBreakID);
      tunneInput = new DigitalInput(Constants.BeamBreakConstants.tunnelBeamBreakID);
    }
  }

  @Override
  public void updateInputs(NoteTrackerIOInputs inputs) {
    // returns true when note doesn't break beam
    inputs.tunnelBeamBreak = tunneInput.get();
    inputs.intakeBeamBreak = intakeInput.get();
  }
}
