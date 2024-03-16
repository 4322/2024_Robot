package frc.robot.subsystems.limelight;

import frc.robot.subsystems.limelight.Limelight.CamMode;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {
  @AutoLog
  public class LimelightIOInputs {
    public double tx;
    public double ty;
    public double ta;
    public double tv;
    public double ledMode;
    public double camMode;
    public double pipeline;

    public int currentPipeline;
  }

  public default void updateInputs(LimelightIOInputs inputs, Limelight limelight) {}

  public default void setCamMode(CamMode mode) {}

  public default void activateRetroReflective() {}

  public default void activateAprilTag() {}

  public default void switchPipeline(int pipelineIdx) {}
}
