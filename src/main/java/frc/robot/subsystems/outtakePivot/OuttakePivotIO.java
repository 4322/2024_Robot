package frc.robot.subsystems.outtakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakePivotIO {
    @AutoLog
    public class OuttakePivotIOInputs {
        public double pivotRotations = 0.0;
        public double pivotRotationsPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempC = 0.0;
        public boolean pivotIsAlive = false;
    
        public double pivotEncoderRotations = 0.0;
        public double pivotEncoderRotationsPerSec = 0.0;
    }

    public default void updateInputs(OuttakePivotIOInputs inputs) {};
    public default boolean initPivot() {return false;}
    public default void setPivotTarget(double rotations) {}
    public default void setBrakeMode() {};
    public default void setCoastMode() {};
    public default void stopPivot() {};

}
