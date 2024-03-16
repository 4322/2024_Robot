package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
  private final TalonFX intake;
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
    intake =
        new TalonFX(IntakeConstants.intakeMotorID, Constants.DriveConstants.Drive.canivoreName);
    deploy =
        new TalonFX(IntakeConstants.deployMotorID, Constants.DriveConstants.Drive.canivoreName);
    deployEncoder = new Canandcoder(IntakeConstants.deployEncoderID);

    configDeploy();
    configIntake();
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

  private void configIntake() {

    intake.getConfigurator().apply(new TalonFXConfiguration());

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

    intake.getConfigurator().apply(hardwareLimitSwitchConfigs);
    intake.getConfigurator().apply(currentLimitsConfigs);
    intake.getConfigurator().apply(motorOutputConfigs);

    intake
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
    inputs.intakeRotations = intake.getPosition().getValue();
    inputs.intakeRotationsPerSec = intake.getVelocity().getValue();
    inputs.intakeAppliedVolts =
        intake.getDutyCycle().getValue() / 2 * intake.getSupplyVoltage().getValue();
    inputs.intakeSupplyCurrentAmps = intake.getSupplyCurrent().getValue();
    inputs.intakeStatorCurrentAmps = intake.getStatorCurrent().getValue();
    inputs.intakeTempC = intake.getDeviceTemp().getValue();
    inputs.intakeIsAlive = intake.isAlive();
    inputs.intakeSpeedPct = intake.get();
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
    intake.setVoltage(voltage);
  }

  @Override
  public void setDeployVoltage(double voltage) {
    deploy.setControl(new VoltageOut(voltage));
    Logger.recordOutput(IntakeConstants.Logging.deployerKey + "voltage", voltage);
  }

  @Override
  public void setIntakeBrakeMode() {
    intake.setNeutralMode(NeutralModeValue.Brake);
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
    intake.setNeutralMode(NeutralModeValue.Coast);
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
    intake.stopMotor();
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
