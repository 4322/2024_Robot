package frc.robot.subsystems.tunnel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.TunnelConstants;
import org.littletonrobotics.junction.Logger;

public class TunnelIOReal implements TunnelIO {
  private TalonFX tunnel;

  public TunnelIOReal() {
    tunnel =
        new TalonFX(TunnelConstants.tunnelMotorID, Constants.DriveConstants.Drive.canivoreName);
    configTalon();
  }

  private void configTalon() {
    tunnel.getConfigurator().apply(new TalonFXConfiguration());
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();

    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    voltageConfigs.PeakForwardVoltage = TunnelConstants.peakVoltage;
    voltageConfigs.PeakReverseVoltage = -TunnelConstants.peakVoltage;
    currentLimitsConfigs.StatorCurrentLimitEnable = TunnelConstants.statorEnabled;
    currentLimitsConfigs.StatorCurrentLimit = TunnelConstants.statorLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = TunnelConstants.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentLimit = TunnelConstants.supplyLimit;

    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    tunnel.getConfigurator().apply(hardwareLimitSwitchConfigs);
    tunnel.getConfigurator().apply(currentLimitsConfigs);
    tunnel.getConfigurator().apply(motorOutputConfigs);
    tunnel.getConfigurator().apply(voltageConfigs);
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {
    inputs.tunnelRotations = tunnel.getPosition().getValue();
    inputs.tunnelRotationsPerSec = tunnel.getVelocity().getValue();
    inputs.tunnelAppliedVolts =
        tunnel.getDutyCycle().getValue() / 2 * tunnel.getSupplyVoltage().getValue();
    inputs.tunnelSupplyCurrentAmps = tunnel.getSupplyCurrent().getValue();
    inputs.tunnelStatorCurrentAmps = tunnel.getStatorCurrent().getValue();
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
