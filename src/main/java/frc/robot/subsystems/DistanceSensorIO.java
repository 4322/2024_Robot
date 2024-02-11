package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface DistanceSensorIO {
    @AutoLog
    public static class DistanceSensorIOInputs {
        public double tunnelDistance = 0.0;
        public boolean tunnelBeamBreak = false;

        public double intakeDistance = 0.0;
        public boolean intakeBeamBreak = false;
    }
    
    public default void updateInputs(DistanceSensorIOInputs inputs) {}
}
