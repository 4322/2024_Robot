package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.DeployConfig;

import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX rightIntakeMotor;
  private final TalonFX leftIntakeMotor;
  private final TalonFX deploy;
  private final Canandcoder deployEncoder;
  // Shuffleboard
  ShuffleboardTab tab;
  GenericEntry intakeFeederVoltage;
  GenericEntry intakeEjectVoltage;
  GenericEntry deployPosition;
  GenericEntry intakeRPS;
  GenericEntry deployMaxRotationsPerSec;
  GenericEntry kP;
  GenericEntry slowPos;
  GenericEntry deployerRPS;

  public IntakeIOReal() {
    rightIntakeMotor =
        new TalonFX(IntakeConstants.rightIntakeMotorID, Constants.DriveConstants.Drive.canivoreName);
    leftIntakeMotor = 
        new TalonFX(IntakeConstants.leftIntakeMotorID, Constants.DriveConstants.Drive.canivoreName);
    deploy =
        new TalonFX(IntakeConstants.deployMotorID, Constants.DriveConstants.Drive.canivoreName);
    deployEncoder = new Canandcoder(IntakeConstants.deployEncoderID);

    configDeploy();
    configIntake(rightIntakeMotor);
    configIntake(leftIntakeMotor);
    rightIntakeMotor.setControl(new Follower(leftIntakeMotor.getDeviceID(), true));
    if (Constants.debug) {
      tab = Shuffleboard.getTab("Intake");
      intakeFeederVoltage =
          tab.add("Intake Feeder Voltage", IntakeConstants.IntakeConfig.intakeFeedVoltage)
              .withSize(1, 1)
              .withPosition(0, 0)
              .getEntry();
      intakeEjectVoltage =
          tab.add("Intake Eject Voltage", IntakeConstants.IntakeConfig.intakeEjectVoltage)
              .withSize(1, 1)
              .withPosition(1, 0)
              .getEntry();
      intakeRPS = tab.add("Intake RPS", 0).withSize(1, 1).withPosition(2, 0).getEntry();
      deployPosition = tab.add("Deployer position", 0).withSize(1, 1).withPosition(0, 1).getEntry();
      deployerRPS = tab.add("Deployer RPS", 0).withPosition(1, 1).withSize(1, 1).getEntry();
      deployMaxRotationsPerSec =
          tab.add("Deployer Max RPS", DeployConfig.maxRotationsPerSec)
              .withPosition(0, 2)
              .withSize(1, 1)
              .getEntry();
      kP = tab.add("Deployer kP", DeployConfig.kP).withPosition(1, 2).withSize(1, 1).getEntry();
      slowPos =
          tab.add("Deployer slowing position", DeployConfig.slowPos)
              .withPosition(2, 2)
              .withSize(1, 1)
              .getEntry();
    }
  }

  private void configIntake(TalonFX talon) {
    talon.getConfigurator().apply(new TalonFXConfiguration());

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();

    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    currentLimitsConfigs.StatorCurrentLimitEnable =
        Constants.IntakeConstants.IntakeConfig.statorEnabled;
    currentLimitsConfigs.StatorCurrentLimit = Constants.IntakeConstants.IntakeConfig.statorLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable =
        Constants.IntakeConstants.IntakeConfig.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.IntakeConstants.IntakeConfig.supplyLimit;

    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    talon.getConfigurator().apply(hardwareLimitSwitchConfigs);
    talon.getConfigurator().apply(currentLimitsConfigs);
    talon.getConfigurator().apply(motorOutputConfigs);

    talon
        .getVelocity()
        .setUpdateFrequency(
            IntakeConstants.IntakeConfig.updateHz, IntakeConstants.IntakeConfig.timeoutMs);
  }

  private void configDeploy() {
    deploy.getConfigurator().apply(new TalonFXConfiguration());

    Slot0Configs slot0Configs = new Slot0Configs();
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = DeployConfig.openLoopRamp;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.DutyCycleNeutralDeadband = 0;
    currentLimitsConfigs.StatorCurrentLimitEnable = DeployConfig.statorEnabled;
    currentLimitsConfigs.StatorCurrentLimit = DeployConfig.statorLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = DeployConfig.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentLimit = DeployConfig.supplyLimit;
    voltageConfigs.PeakForwardVoltage = DeployConfig.peakForwardVoltage;
    voltageConfigs.PeakReverseVoltage = DeployConfig.peakReverseVoltage;

    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    deploy.getConfigurator().apply(hardwareLimitSwitchConfigs);
    deploy.getConfigurator().apply(currentLimitsConfigs);
    deploy.getConfigurator().apply(slot0Configs);
    deploy.getConfigurator().apply(openLoopRampsConfigs);
    deploy.getConfigurator().apply(voltageConfigs);
    deploy.getConfigurator().apply(motorOutputConfigs);
    deploy.getConfigurator().apply(softwareLimitSwitchConfigs);

    // don't need rapid position update because we use the abs encoder
    deploy.getPosition().setUpdateFrequency(DeployConfig.updateHz, DeployConfig.timeoutMs);

    // zero helium abs encoder on the floor
    // fully retracted abs position is 0.6 rotations
    Canandcoder.Settings settings = new Canandcoder.Settings();
    settings.setInvertDirection(true);
    settings.setPositionFramePeriod(0.010);
    settings.setVelocityFramePeriod(0.010);
    settings.setStatusFramePeriod(1.0);
    deployEncoder.setSettings(settings, 0.050);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rightIntakeRotations = rightIntakeMotor.getPosition().getValue();
    inputs.rightIntakeRotationsPerSec = rightIntakeMotor.getVelocity().getValue();
    inputs.rightIntakeAppliedVolts =
        rightIntakeMotor.getDutyCycle().getValue() / 2 * rightIntakeMotor.getSupplyVoltage().getValue();
    inputs.rightIntakeSupplyCurrentAmps = rightIntakeMotor.getSupplyCurrent().getValue();
    inputs.rightIntakeStatorCurrentAmps = rightIntakeMotor.getStatorCurrent().getValue();
    inputs.rightIntakeTempC = rightIntakeMotor.getDeviceTemp().getValue();
    inputs.rightIntakeIsAlive = rightIntakeMotor.isAlive();
    inputs.rightIntakeSpeedPct = rightIntakeMotor.get();

    inputs.leftIntakeRotations = leftIntakeMotor.getPosition().getValue();
    inputs.leftIntakeRotationsPerSec = leftIntakeMotor.getVelocity().getValue();
    inputs.leftIntakeAppliedVolts =
        leftIntakeMotor.getDutyCycle().getValue() / 2 * leftIntakeMotor.getSupplyVoltage().getValue();
    inputs.leftIntakeSupplyCurrentAmps = leftIntakeMotor.getSupplyCurrent().getValue();
    inputs.leftIntakeStatorCurrentAmps = leftIntakeMotor.getStatorCurrent().getValue();
    inputs.leftIntakeTempC = leftIntakeMotor.getDeviceTemp().getValue();
    inputs.leftIntakeIsAlive = leftIntakeMotor.isAlive();
    inputs.leftIntakeSpeedPct = leftIntakeMotor.get();

    inputs.deployRotations = deploy.getPosition().getValue();
    inputs.deployRotationsPerSec = deploy.getVelocity().getValue();
    inputs.deployAppliedVolts =
        deploy.getDutyCycle().getValue() / 2 * deploy.getSupplyVoltage().getValue();
    inputs.deploySupplyCurrentAmps = deploy.getSupplyCurrent().getValue();
    inputs.deployStatorCurrentAmps = deploy.getStatorCurrent().getValue();
    inputs.deployTempC = deploy.getDeviceTemp().getValue();
    inputs.deployIsAlive = deploy.isAlive();

    inputs.heliumAbsRotations = deployEncoder.getAbsPosition();
    inputs.heliumRPS = deployEncoder.getVelocity();

    inputs.deployAppliedControl = deploy.getAppliedControl().toString();
    
    // If intake deployer is above threshold at 0.95 rotations, then assume it is below zero 
    // point and as a result wraps back up to 1.0 rotations or value close to it.
    // If this scenario occurs, then set abs value position back to 0 rotations
    if (inputs.heliumAbsRotations
        > Constants.EncoderInitializeConstants.absEncoderMaxZeroingThreshold) {
      inputs.heliumAbsRotations = 0;
    }

    if (Constants.debug) {
      inputs.intakeFeederVoltage =
          intakeFeederVoltage.getDouble(IntakeConstants.IntakeConfig.intakeFeedVoltage);
      inputs.intakeEjectVoltage =
          intakeEjectVoltage.getDouble(IntakeConstants.IntakeConfig.intakeEjectVoltage);
      inputs.deployMaxRotationsPerSec =
          deployMaxRotationsPerSec.getDouble(DeployConfig.maxRotationsPerSec);
      inputs.deployKp = kP.getDouble(DeployConfig.kP);
      inputs.slowPos = slowPos.getDouble(DeployConfig.slowPos);
      deployerRPS.setDouble(inputs.heliumRPS);
    } else {
      inputs.intakeFeederVoltage = IntakeConstants.IntakeConfig.intakeFeedVoltage;
      inputs.intakeEjectVoltage = IntakeConstants.IntakeConfig.intakeEjectVoltage;
      inputs.deployMaxRotationsPerSec = DeployConfig.maxRotationsPerSec;
      inputs.deployKp = DeployConfig.kP;
      inputs.slowPos = DeployConfig.slowPos;
    }
  }

  @Override
  public void setFeedingVoltage(double voltage) {
    rightIntakeMotor.setVoltage(voltage);
  }

  @Override
  public void setDeployVoltage(double voltage) {
    deploy.setControl(new VoltageOut(voltage));
    Logger.recordOutput(IntakeConstants.Logging.deployerKey + "voltage", voltage);
  }

  @Override
  public void setIntakeBrakeMode() {
    rightIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    Logger.recordOutput(IntakeConstants.Logging.feederHardwareOutputsKey + "NeutralMode", "Brake");
  }

  @Override
  public void setDeployerBrakeMode() {
    deploy.setNeutralMode(NeutralModeValue.Brake);
    Logger.recordOutput(
        IntakeConstants.Logging.deployerHardwareOutputsKey + "NeutralMode", "Brake");
  }

  @Override
  public void setIntakeCoastMode() {
    rightIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
    Logger.recordOutput(IntakeConstants.Logging.feederHardwareOutputsKey + "NeutralMode", "Coast");
  }

  @Override
  public void setDeployerCoastMode() {
    deploy.setNeutralMode(NeutralModeValue.Coast);
    Logger.recordOutput(
        IntakeConstants.Logging.deployerHardwareOutputsKey + "NeutralMode", "Coast");
  }

  @Override
  public void stopFeeder() {
    rightIntakeMotor.stopMotor();
  }

  @Override
  public void stopDeployer() {
    deploy.stopMotor();
  }

  @Override
  public void setDeployKp(double kP) {
    Slot0Configs configs = new Slot0Configs();
    configs.kP = kP;
    deploy.getConfigurator().refresh(configs);
  }
}
