package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.limelight.Limelight.CamMode;

public interface LimelightIO {
    @AutoLog
    public class LimelightIOInputs{
        public NetworkTableEntry tx;
        public NetworkTableEntry ty;
        public NetworkTableEntry ta;
        public NetworkTableEntry tv;
        public NetworkTableEntry ledMode;
        public NetworkTableEntry camMode;
        public NetworkTableEntry pipeline;

        public int currentPipeline;
    }
    
    public default void updateInputs(LimelightIOInputs inputs) {}

    public default void setCamMode(CamMode mode) {}

    public default void activateRetroReflective() {}

    public default void activateAprilTag() {}

    public default void switchPipeline(int pipelineIdx) {}
}
