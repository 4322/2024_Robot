package frc.robot.subsystems;

public class DistanceSensorIOReal implements DistanceSensorIO {
    
    public DistanceSensorIOReal() {
        // TODO: initialize sensors
    }
    
    @Override
    public void updateInputs(DistanceSensorIOInputs inputs) {
        inputs.tunnelDistance = 0.0; // TODO
        inputs.tunnelBeamBreak = false; // TODO

        inputs.intakeDistance = 0.0; // TODO
        inputs.intakeBeamBreak = false; // TODO
    }
}
