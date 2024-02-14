package frc.robot.subsystems.tunnel;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.OuttakeConstants.kP;
    slot0Configs.kI = Constants.OuttakeConstants.kI;
    slot0Configs.kD = Constants.OuttakeConstants.kD;
    slot0Configs.kV = Constants.OuttakeConstants.kF;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        Constants.OuttakeConstants.closedLoopRampSec;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = Constants.OuttakeConstants.openLoopRampSec;
    // bottomOuttakeMotor.setControl(new
    // Follower(topOuttakeMotor.getDeviceID(),true));
    tunnel.getConfigurator().apply(slot0Configs);
    tunnel.getConfigurator().apply(closedLoopRampsConfigs);
    tunnel.getConfigurator().apply(openLoopRampsConfigs);
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
  public void setTunnel(double desiredVelocityRPS) {
    tunnel.setControl(new VelocityVoltage(desiredVelocityRPS));
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
