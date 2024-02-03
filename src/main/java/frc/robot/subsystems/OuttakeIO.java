package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    @AutoLog
    public class OuttakeIOInputs
    {
        public double topCurrentAmps = 0.0;
        public double topTempC = 0.0;
        public double topRotationsPerSec = 0.0;

        public double bottomCurrentAmps = 0.0;
        public double bottomTempC = 0.0;
        public double bottomRotationsPerSec = 0.0;

        public double pivotRotations = 0.0;
        public double pivotRotationsPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempC = 0.0;
        public boolean pivotIsAlive = false;
    
        public double pivotEncoderRotations = 0.0;
        public double pivotEncoderRotationsPerSec = 0.0;
        public boolean topOuttakeIsAlive = false;
        public boolean bottomOuttakeIsAlive = false;
    }
    
    public default void updateInputs(OuttakeIOInputs inputs) {};
    public default boolean initPivot() {return false;}
    public default void setPivotTarget(double rotations) {}
    public default void setOuttakeRPM(double desiredTopVelocityRPM, double desiredBottomVelocityRPM) {};
    public default void setBrakeMode() {};
    public default void setCoastMode() {};
    public default void stopOuttake() {};
    public default void stopPivot() {};
}