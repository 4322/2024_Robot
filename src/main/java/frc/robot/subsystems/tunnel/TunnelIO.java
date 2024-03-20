package frc.robot.subsystems.tunnel;

import org.littletonrobotics.junction.AutoLog;

public interface TunnelIO {
  @AutoLog
  public class TunnelIOInputs {
    public double tunnelRotations = 0.0;
    public double tunnelRotationsPerSec = 0.0;
    public double tunnelAppliedVolts = 0.0;
    public double tunnelSupplyCurrentAmps = 0.0;
    public double tunnelStatorCurrentAmps = 0.0;
    public double tunnelTempC = 0.0;
    public boolean tunnelIsAlive = false;
    public double tunnelSpeedPct = 0.0;
  }

  public default void updateInputs(TunnelIOInputs inputs) {}

  public default void setTunnel(double voltage) {}

  public default void setBrakeMode() {}

  public default void setCoastMode() {}

  public default void stopTunnel() {}
}
