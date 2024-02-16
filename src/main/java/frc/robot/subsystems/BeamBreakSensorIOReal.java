package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class BeamBreakSensorIOReal implements BeamBreakSensorIO {
  DigitalInput intakeInput;
  DigitalInput tunneInput;

  public BeamBreakSensorIOReal() {
    if (Constants.sensorsEnabled) {
      intakeInput = new DigitalInput(Constants.BeakBreakConstants.intakeBeamBreakID);
      tunneInput = new DigitalInput(Constants.BeakBreakConstants.tunnelBeamBreakID);
    }
  }

  @Override
  public void updateInputs(BeamBreakSensorIOInputs inputs) {
    // returns true when note doesn't break beam
    inputs.tunnelBeamBreak = tunneInput.get();
    inputs.intakeBeamBreak = intakeInput.get();
  }
}
