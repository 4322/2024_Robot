package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX intake;
  private final TalonFX deploy;
  private final Canandcoder deployEncoder;
  //Shuffleboard
  ShuffleboardTab tab;
  GenericEntry intakeFeederVoltage;
  GenericEntry intakeEjectVoltage;
  GenericEntry deployPositionRotations;
  GenericEntry retractPositionRotations;
  GenericEntry deployPosition;
  GenericEntry deployerRPS;
  GenericEntry isCoasting;
  GenericEntry flywheelRPS;
  public IntakeIOReal() {
    intake = new TalonFX(IntakeConstants.intakeMotorID, Constants.DriveConstants.Drive.canivoreName);
    deploy = new TalonFX(IntakeConstants.deployMotorID, Constants.DriveConstants.Drive.canivoreName);
    deployEncoder = new Canandcoder(IntakeConstants.deployEncoderID);

    configDeploy();
    configIntake();
    if(Constants.debug)
    {
      tab = Shuffleboard.getTab("Intake");
      intakeFeederVoltage = tab.add("Intake Feeder Voltage",IntakeConstants.IntakeConfig.intakeFeedVoltage).withSize(1,1).withPosition(0,0).getEntry();
      intakeEjectVoltage = tab.add("Intake Eject Voltage",IntakeConstants.IntakeConfig.intakeEjectVoltage).withSize(1,1).withPosition(1,0).getEntry();
      flywheelRPS = tab.add("Intake flywheel RPS", 0).withSize(1, 1).withPosition(2, 0).getEntry();
      deployPositionRotations = tab.add("Deployer Target (Rotations)", 99999).withSize(1,1).withPosition(3, 0).getEntry();
      retractPositionRotations = tab.add("Retract Deployer Target (Rotations)", IntakeConstants.Deploy.retractPositionRotations).withSize(1, 1).withPosition(0, 1).getEntry();
      deployPosition = tab.add("Deployer position",0).withSize(1, 1).withPosition(1,1).getEntry();
      deployerRPS = tab.add("Max Deployer RPS",IntakeConstants.Deploy.maxVelRotationsPerSec).withSize(1,1).withPosition(2,1).getEntry();
      isCoasting = tab.add("Are Intake Motors in Coast Mode? (Read Only)", IntakeConstants.IntakeConfig.neutralMode == NeutralModeValue.Coast).withWidget(BuiltInWidgets.kBooleanBox).withSize(3,1).withPosition(1,1).getEntry();
    }
  }

  private void configIntake() {

    intake.getConfigurator().apply(new TalonFXConfiguration());

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = IntakeConstants.IntakeConfig.neutralMode;

    intake.getConfigurator().apply(motorOutputConfigs);

    intake
        .getVelocity()
        .setUpdateFrequency(
            IntakeConstants.IntakeConfig.updateHz, IntakeConstants.IntakeConfig.timeoutMs);
  }

  private void configDeploy() {
    deploy.getConfigurator().apply(new TalonFXConfiguration());

    Slot0Configs slot0Configs = new Slot0Configs();
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    slot0Configs.kP = IntakeConstants.DeployConfig.kP;
    slot0Configs.kD = IntakeConstants.DeployConfig.kD;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        IntakeConstants.DeployConfig.configCLosedLoopRamp;
    voltageConfigs.PeakForwardVoltage = IntakeConstants.DeployConfig.maxVoltage;
    voltageConfigs.PeakReverseVoltage = -IntakeConstants.DeployConfig.maxVoltage;
    motorOutputConfigs.NeutralMode = IntakeConstants.DeployConfig.neutralMode;

    deploy.getConfigurator().apply(slot0Configs);
    deploy.getConfigurator().apply(closedLoopRampsConfigs);
    deploy.getConfigurator().apply(voltageConfigs);
    deploy.getConfigurator().apply(motorOutputConfigs);

    // don't need rapid position update
    deploy
        .getPosition()
        .setUpdateFrequency(
            IntakeConstants.DeployConfig.updateHz, IntakeConstants.DeployConfig.timeoutMs);
  }

  public void updateShuffleboard()
  {
    flywheelRPS.setDouble(intake.getVelocity().getValue() / 60);
    deployPosition.setDouble(deploy.getPosition().getValue());
  }
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRotations = intake.getPosition().getValue();
    inputs.intakeRotationsPerSec = intake.getVelocity().getValue() / 60;
    inputs.intakeAppliedVolts =
        intake.getDutyCycle().getValue() / 2 * intake.getSupplyVoltage().getValue();
    inputs.intakeCurrentAmps = intake.getSupplyCurrent().getValue();
    inputs.intakeTempC = intake.getDeviceTemp().getValue();
    inputs.intakeIsAlive = intake.isAlive();
    inputs.intakeSpeedPct = intake.get();
    inputs.deployRotations = deploy.getPosition().getValue();
    inputs.deployRotationsPerSec = deploy.getVelocity().getValue() / 60;
    inputs.deployAppliedVolts =
        deploy.getDutyCycle().getValue() / 2 * deploy.getSupplyVoltage().getValue();
    inputs.deployCurrentAmps = deploy.getSupplyCurrent().getValue();
    inputs.deployTempC = deploy.getDeviceTemp().getValue();
    inputs.deployIsAlive = deploy.isAlive();

    inputs.deployEncoderRotations = deployEncoder.getPosition();
    inputs.deployEncoderRotationsPerSec = deployEncoder.getVelocity();

    inputs.deployAppliedControl = deploy.getAppliedControl().toString();
    if(Constants.debug)
    {
      inputs.intakeFeederVoltage = intakeFeederVoltage.getDouble(IntakeConstants.IntakeConfig.intakeFeedVoltage);
      inputs.intakeEjectVoltage = intakeEjectVoltage.getDouble(IntakeConstants.IntakeConfig.intakeEjectVoltage);
      inputs.deployPositionRotations = deployPositionRotations.getDouble(IntakeConstants.Deploy.deployPositionRotations);
      inputs.retractPositionRotations = retractPositionRotations.getDouble(IntakeConstants.Deploy.retractPositionRotations);
      updateShuffleboard();
    }
    else
    {
      inputs.intakeFeederVoltage = IntakeConstants.IntakeConfig.intakeFeedVoltage;
      inputs.intakeEjectVoltage = IntakeConstants.IntakeConfig.intakeEjectVoltage;
      inputs.deployPositionRotations = IntakeConstants.Deploy.deployPositionRotations;
      inputs.retractPositionRotations = IntakeConstants.Deploy.retractPositionRotations;
    }
  }

  @Override
  public void setFeedingVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  @Override
  public boolean initMotorPos() {
    deploy.setPosition(
        deployEncoder.getAbsPosition() * IntakeConstants.Deploy.encoderGearReduction);
    return OrangeMath.equalToTwoDecimal(deployEncoder.getVelocity(), 0);
  }

  @Override
  public void setDeployTarget(double rotations) {
    if(Constants.debug)
    {
      deploy.setControl(
        new PositionVoltage(
          rotations,
          deployerRPS.getDouble(IntakeConstants.Deploy.maxVelRotationsPerSec),
          IntakeConstants.Deploy.enableFOC,
          IntakeConstants.Deploy.FF,
          IntakeConstants.Deploy.positionVoltageSlot,
          IntakeConstants.Deploy.overrideBrakeDuringNeutral,
          IntakeConstants.Deploy.limitForwardMotion,
          IntakeConstants.Deploy.limitReverseMotion));
    }
    else
    {
      deploy.setControl(
        new PositionVoltage(
          rotations,
          IntakeConstants.Deploy.maxVelRotationsPerSec,
          IntakeConstants.Deploy.enableFOC,
          IntakeConstants.Deploy.FF,
          IntakeConstants.Deploy.positionVoltageSlot,
          IntakeConstants.Deploy.overrideBrakeDuringNeutral,
          IntakeConstants.Deploy.limitForwardMotion,
           IntakeConstants.Deploy.limitReverseMotion));
    }
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intake.getConfigurator().refresh(motorOutputConfigs);
    deploy.getConfigurator().refresh(motorOutputConfigs);
    if(Constants.debug)
    {
      isCoasting.setBoolean(false);
    }
    Logger.recordOutput(IntakeConstants.Logging.feederHardwareOutputsKey + "NeutralMode", "Brake");
    Logger.recordOutput(
        IntakeConstants.Logging.deployerHardwareOutputsKey + "NeutralMode", "Brake");
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    intake.getConfigurator().refresh(motorOutputConfigs);
    deploy.getConfigurator().refresh(motorOutputConfigs);
    if(Constants.debug)
    {
      isCoasting.setBoolean(true);
    }
    Logger.recordOutput(IntakeConstants.Logging.feederHardwareOutputsKey + "NeutralMode", "Coast");
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
}
