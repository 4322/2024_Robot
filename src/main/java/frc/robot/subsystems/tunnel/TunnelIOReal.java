package frc.robot.subsystems.tunnel;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.TunnelConstants;
import org.littletonrobotics.junction.Logger;

public class TunnelIOReal implements TunnelIO {
  private TalonFX tunnel;

  public TunnelIOReal() {
    tunnel = new TalonFX(TunnelConstants.tunnelMotorID);
    configTalon();
  }

  private void configTalon() {
    tunnel.getConfigurator().apply(new TalonFXConfiguration());
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();

    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    voltageConfigs.PeakForwardVoltage = TunnelConstants.peakVoltage;
    voltageConfigs.PeakReverseVoltage = -TunnelConstants.peakVoltage;
    tunnel.getConfigurator().apply(motorOutputConfigs);
    tunnel.getConfigurator().apply(voltageConfigs);
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {
    inputs.tunnelRotations = tunnel.getPosition().getValue();
    inputs.tunnelRotationsPerSec = tunnel.getVelocity().getValue() / 60;
    inputs.tunnelAppliedVolts =
        tunnel.getDutyCycle().getValue() / 2 * tunnel.getSupplyVoltage().getValue();
    inputs.tunnelCurrentAmps = tunnel.getSupplyCurrent().getValue();
    inputs.tunnelTempC = tunnel.getDeviceTemp().getValue();
    inputs.tunnelIsAlive = tunnel.isAlive();
    inputs.tunnelSpeedPct = tunnel.get();
  }

  @Override
  public void setTunnel(double desiredVolts) {
    tunnel.setControl(new VoltageOut(desiredVolts));
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    tunnel.getConfigurator().refresh(motorOutputConfigs);

    Logger.recordOutput(TunnelConstants.Logging.hardwareOutputsKey + "NeutralMode", "Brake");
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    tunnel.getConfigurator().refresh(motorOutputConfigs);

    Logger.recordOutput(TunnelConstants.Logging.hardwareOutputsKey + "NeutralMode", "Coast");
  }

  @Override
  public void stopTunnel() {
    tunnel.stopMotor();
  }
}
