package frc.robot.subsystems.tunnel;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
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

    Slot0Configs slot0Configs = new Slot0Configs();
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    slot0Configs.kP = Constants.TunnelConstants.TunnelConfig.kP;
    slot0Configs.kD = Constants.TunnelConstants.TunnelConfig.kD;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        TunnelConstants.TunnelConfig.configClosedLoopRamp;
    voltageConfigs.PeakForwardVoltage = TunnelConstants.TunnelConfig.maxVoltage;
    voltageConfigs.PeakReverseVoltage = -TunnelConstants.TunnelConfig.maxVoltage;
    motorOutputConfigs.NeutralMode = TunnelConstants.TunnelConfig.neutralMode;

    tunnel.getConfigurator().apply(slot0Configs);
    tunnel.getConfigurator().apply(motorOutputConfigs);
    tunnel.getConfigurator().apply(closedLoopRampsConfigs);
    tunnel.getConfigurator().apply(voltageConfigs);

    // don't need rapid position update
    tunnel
        .getPosition()
        .setUpdateFrequency(
            TunnelConstants.TunnelConfig.updateHz,
            TunnelConstants.TunnelConfig.timeoutMs);
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
  public void setTunnelTarget(double rotations) {
    tunnel.setControl(
      new PositionVoltage(
        rotations,
        TunnelConstants.Feed.maxVelRotationsPerSec,
        TunnelConstants.Feed.enableFOC,
        TunnelConstants.Feed.FF,
        TunnelConstants.Feed.positionVoltageSlot,
        TunnelConstants.Feed.overrideBrakeDuringNeutral,
        TunnelConstants.Feed.limitForwardMotion,
        TunnelConstants.Feed.limitReverseMotion));
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
