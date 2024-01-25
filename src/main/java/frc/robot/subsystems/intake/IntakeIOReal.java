package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;
import frc.utility.OrangeMath;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX intake;
  private final TalonFX deploy;
  private final CANcoder deployEncoder;

  public IntakeIOReal() {
    intake = new TalonFX(IntakeConstants.intakeMotorID);
    deploy = new TalonFX(IntakeConstants.deployMotorID);
    deployEncoder = new CANcoder(IntakeConstants.deployEncoderID);
    configIntake();
    configDeploy();
    configEncoder();
  }

  private void configIntake() {
    intake.getConfigurator().apply(new TalonFXConfiguration());

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.NeutralMode = IntakeConstants.IntakeConfig.neutralMode;

    intake.getConfigurator().apply(motorOutputConfigs);

    intake.getVelocity().setUpdateFrequency(IntakeConstants.IntakeConfig.updateHz,
        IntakeConstants.IntakeConfig.timeoutMs);
  }

  private void configDeploy() {
    deploy.getConfigurator().apply(new TalonFXConfiguration());

    Slot0Configs slot0Configs = new Slot0Configs();
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    slot0Configs.kP = IntakeConstants.DeployConfig.kP;
    slot0Configs.kD = IntakeConstants.DeployConfig.kD;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = IntakeConstants.DeployConfig.configCLosedLoopRamp;
    voltageConfigs.PeakForwardVoltage = IntakeConstants.DeployConfig.maxVoltage;
    voltageConfigs.PeakReverseVoltage = -IntakeConstants.DeployConfig.maxVoltage;
    motorOutputConfigs.NeutralMode = IntakeConstants.DeployConfig.neutralMode;

    deploy.getConfigurator().apply(slot0Configs);
    deploy.getConfigurator().apply(closedLoopRampsConfigs);
    deploy.getConfigurator().apply(voltageConfigs);
    deploy.getConfigurator().apply(motorOutputConfigs);

    // don't need rapid position update
    deploy.getPosition().setUpdateFrequency(IntakeConstants.DeployConfig.updateHz,
        IntakeConstants.DeployConfig.timeoutMs);
  }

  private void configEncoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor.AbsoluteSensorRange = IntakeConstants.EncoderConfig.absSensorRange;
    deployEncoder.getConfigurator().apply(canConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRotations = intake.getPosition().getValue();
    inputs.intakeRotationsPerSec = intake.getVelocity().getValue() / 60;
    inputs.intakeAppliedVolts = intake.getDutyCycle().getValue() / 2 * intake.getSupplyVoltage().getValue();
    inputs.intakeCurrentAmps = intake.getSupplyCurrent().getValue();
    inputs.intakeTempC = intake.getDeviceTemp().getValue();
    inputs.intakeIsAlive = intake.isAlive();

    inputs.deployRotations = deploy.getPosition().getValue();
    inputs.deployRotationsPerSec = deploy.getVelocity().getValue() / 60;
    inputs.deployAppliedVolts = deploy.getDutyCycle().getValue() / 2 * deploy.getSupplyVoltage().getValue();
    inputs.deployCurrentAmps = deploy.getSupplyCurrent().getValue();
    inputs.deployTempC = deploy.getDeviceTemp().getValue();
    inputs.deployIsAlive = deploy.isAlive();

    inputs.deployEncoderRotations = deployEncoder.getPosition().getValue();
    inputs.deployEncoderRotationsPerSec = deployEncoder.getVelocity().getValue() / 60;

    inputs.intakeSpeedPct = intake.get();
    inputs.deployAppliedControl = deploy.getAppliedControl().toString();
  }

  @Override
  public boolean initMotorPos() {
    deploy.setPosition(deployEncoder.getAbsolutePosition().getValue());
    return OrangeMath.equalToTwoDecimal(deployEncoder.getVelocity().getValue(), 0);
  }

  @Override
  public void setIntake(double pct) {
    intake.set(pct);
  }

  @Override
  public void setDeployTarget(double rotations) {
    deploy.setControl(new PositionVoltage(rotations, IntakeConstants.Deploy.maxVelRotationsPerSec,
        IntakeConstants.Deploy.enableFOC, IntakeConstants.Deploy.FF, IntakeConstants.Deploy.positionVoltageSlot,
        IntakeConstants.Deploy.overrideBrakeDuringNeutral, IntakeConstants.Deploy.limitForwardMotion,
        IntakeConstants.Deploy.limitReverseMotion));
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intake.getConfigurator().refresh(motorOutputConfigs);
    deploy.getConfigurator().refresh(motorOutputConfigs);

    Logger.recordOutput(IntakeConstants.Logging.hardwareOutputsKey + "NeutralMode", "Brake");
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    intake.getConfigurator().refresh(motorOutputConfigs);
    deploy.getConfigurator().refresh(motorOutputConfigs);

    Logger.recordOutput(IntakeConstants.Logging.hardwareOutputsKey + "NeutralMode", "Coast");
  }

  @Override
  public void stopIntake() {
    intake.stopMotor();
  }

  @Override
  public void stopDeploy() {
    deploy.stopMotor();
  }

}
