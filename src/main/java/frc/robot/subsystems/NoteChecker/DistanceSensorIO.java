package frc.robot.subsystems.NoteChecker;

import org.littletonrobotics.junction.AutoLog;


public interface DistanceSensorIO {
    public double getDistance();
    
    @AutoLog 
    public static class DistanceSensorIOInputs{
        public double distance = 0.0; 
    }

    public default void updateInputs(DistanceSensorIOInputs inputs) {}
}
