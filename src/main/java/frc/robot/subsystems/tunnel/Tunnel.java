package frc.robot.subsystems.tunnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TunnelConstants;
import org.littletonrobotics.junction.Logger;

public class Tunnel extends SubsystemBase {
  private TunnelIO io;
  private TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();
  private boolean initialized;

  private static Tunnel tunnel;

  public static Tunnel getInstance() {
    if (tunnel == null) {
      tunnel = new Tunnel();
    }
    return tunnel;
  }

  private Tunnel() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.tunnelEnabled) {
          io = new TunnelIOReal();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (io == null) {
      io = new TunnelIO() {};
    }
  }

  @Override
  public void periodic() {
    if (Constants.tunnelEnabled && initialized) {
      io.updateInputs(inputs);
      Logger.processInputs(TunnelConstants.Logging.key, inputs);
    }
  }

  public void feed() { // run the tunnel in the direction of the outtake
    if (Constants.tunnelEnabled && initialized) {
      io.setTunnel(TunnelConstants.turnSpeedPct);
      Logger.recordOutput(
          TunnelConstants.Logging.key + "TunnelTargetSpeedPct", TunnelConstants.turnSpeedPct);
      Logger.recordOutput(TunnelConstants.Logging.key + "TunnelStopped", false);
    }
  }

  public void setBrakeMode() {
    if (Constants.tunnelEnabled && initialized) {
      io.setBrakeMode();
      Logger.recordOutput(TunnelConstants.Logging.key + "TargetBrakeMode", "Brake");
    }
  }

  public void setCoastMode() {
    if (Constants.tunnelEnabled && initialized) {
      io.setCoastMode();
      Logger.recordOutput(TunnelConstants.Logging.key + "TargetBrakeMode", "Coast");
    }
  }

  public void stopTunnel() {
    if (Constants.tunnelEnabled && initialized) {
      io.stopTunnel();
      Logger.recordOutput(TunnelConstants.Logging.key + "TunnelTargetSpeedPct", 0);
      Logger.recordOutput(TunnelConstants.Logging.key + "TunnelStopped", true);
    }
  }
}
