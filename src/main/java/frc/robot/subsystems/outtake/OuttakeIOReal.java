package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class OuttakeIOReal implements OuttakeIO {
  private TalonFX leftOuttakeMotor;
  private TalonFX rightOuttakeMotor;
  private TalonFX pivotMotor;
  private Canandcoder pivotEncoder;
  // shuffleboard
  ShuffleboardTab tab;
  GenericEntry outtakeFlywheelSpeed;
  GenericEntry pivotPosition;
  GenericEntry outtakeOn;
  GenericEntry pivotOn;

  private double heliumAbsoluteRotations;

  public OuttakeIOReal() {
    leftOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.leftOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    rightOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.rightOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    pivotMotor =
        new TalonFX(OuttakeConstants.pivotDeviceID, Constants.DriveConstants.Drive.canivoreName);
    pivotEncoder = new Canandcoder(OuttakeConstants.pivotEncoderID);

    configOuttake(leftOuttakeMotor);
    configOuttake(rightOuttakeMotor);
    rightOuttakeMotor.setControl(new Follower(leftOuttakeMotor.getDeviceID(), true));
    configPivot(pivotMotor);
    if (Constants.outtakeTuningMode) {
      tab = Shuffleboard.getTab("Outtake");
      outtakeFlywheelSpeed =
          tab.add("Desired Flywheel Velocity (RPS)", 0)
              .withSize(1, 1)
              .withPosition(0, 0)
              .getEntry();
      pivotPosition =
          tab.add("Pivot Position (Rotations)", 0).withSize(1, 1).withPosition(1, 0).getEntry();
      outtakeOn = 
          tab.add("Outtake On", false).withSize(1, 1).withPosition(0, 2).getEntry();
      pivotOn =
          tab.add("Pivot On", false).withSize(1,1).withPosition(1, 1).getEntry();
    }
  }

  private void configOuttake(TalonFX talon) {
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.OuttakeConstants.kP;
    slot0Configs.kI = Constants.OuttakeConstants.kI;
    slot0Configs.kD = Constants.OuttakeConstants.kD;
    slot0Configs.kV = Constants.OuttakeConstants.kF;
    slot0Configs.kS = Constants.OuttakeConstants.kS;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        Constants.OuttakeConstants.closedLoopRampSec;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = Constants.OuttakeConstants.openLoopRampSec;
    currentLimitsConfigs.StatorCurrentLimitEnable = Constants.OuttakeConstants.statorEnabled;
    currentLimitsConfigs.StatorCurrentLimit = Constants.OuttakeConstants.shooterStatorLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = Constants.OuttakeConstants.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.OuttakeConstants.shooterSupplyLimit;
    currentLimitsConfigs.SupplyCurrentThreshold =
        Constants.OuttakeConstants.shooterSupplyCurrentThreshold;
    currentLimitsConfigs.SupplyTimeThreshold =
        Constants.OuttakeConstants.shooterSupplyTimeThreshold;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    talon.getConfigurator().apply(hardwareLimitSwitchConfigs);
    talon.getConfigurator().apply(currentLimitsConfigs);
    talon.getConfigurator().apply(slot0Configs);
    talon.getConfigurator().apply(closedLoopRampsConfigs);
    talon.getConfigurator().apply(openLoopRampsConfigs);
    talon.getConfigurator().apply(motorOutputConfigs);
  }

  private void configPivot(TalonFX talon) {
    Slot0Configs slot0Configs = new Slot0Configs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    slot0Configs.kP = Constants.OuttakeConstants.pivotkP;
    slot0Configs.kI = OuttakeConstants.pivotkI;
    slot0Configs.kD = OuttakeConstants.pivotkD;
    voltageConfigs.PeakForwardVoltage = OuttakeConstants.pivotPeakForwardVoltage;
    voltageConfigs.PeakReverseVoltage = OuttakeConstants.pivotPeakReverseVoltage;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = OuttakeConstants.pivotClosedLoopSec;
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = OuttakeConstants.limitForwardMotion;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = OuttakeConstants.limitReverseMotion;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold =
        OuttakeConstants.forwardSoftLimitThresholdRotations;
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold =
        OuttakeConstants.reverseSoftLimitThresholdRotations;
    currentLimitsConfigs.StatorCurrentLimitEnable = Constants.OuttakeConstants.statorEnabled;
    currentLimitsConfigs.StatorCurrentLimit = Constants.OuttakeConstants.pivotStatorLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = Constants.OuttakeConstants.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.OuttakeConstants.pivotSupplyLimit;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    talon.getConfigurator().apply(hardwareLimitSwitchConfigs);
    talon.getConfigurator().apply(currentLimitsConfigs);
    talon.getConfigurator().apply(slot0Configs);
    talon.getConfigurator().apply(closedLoopRampsConfigs);
    talon.getConfigurator().apply(voltageConfigs);
    talon.getConfigurator().apply(motorOutputConfigs);
    talon.getConfigurator().apply(softwareLimitSwitchConfigs);

    // zero helium abs encoder on the stop bar
    // 60 degree shooting angle abs position is about 0.72 rotations
    // the encoder wraps at about 78 degrees
    Canandcoder.Settings settings = new Canandcoder.Settings();
    settings.setInvertDirection(true);
    settings.setPositionFramePeriod(0.010);
    settings.setVelocityFramePeriod(0.050);
    settings.setStatusFramePeriod(1.0);
    pivotEncoder.setSettings(settings, 0.050);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.leftCurrentAmps = leftOuttakeMotor.getSupplyCurrent().getValue();
    inputs.leftTempC = leftOuttakeMotor.getDeviceTemp().getValue();
    inputs.leftRotationsPerSec = leftOuttakeMotor.getVelocity().getValue();
    inputs.leftOuttakeIsAlive = leftOuttakeMotor.isAlive();

    inputs.rightCurrentAmps = rightOuttakeMotor.getSupplyCurrent().getValue();
    inputs.rightTempC = rightOuttakeMotor.getDeviceTemp().getValue();
    inputs.rightRotationsPerSec = rightOuttakeMotor.getVelocity().getValue();
    inputs.rightOuttakeIsAlive = rightOuttakeMotor.isAlive();

    inputs.pivotRotations = pivotMotor.getPosition().getValue();
    inputs.pivotRotationsPerSec = pivotMotor.getVelocity().getValue();
    inputs.pivotAppliedVolts =
        pivotMotor.getDutyCycle().getValue() / 2 * pivotMotor.getSupplyVoltage().getValue();
    inputs.pivotCurrentAmps = pivotMotor.getSupplyCurrent().getValue();
    inputs.pivotTempC = pivotMotor.getDeviceTemp().getValue();
    inputs.pivotIsAlive = pivotMotor.isAlive();

    inputs.heliumAbsRotations = pivotEncoder.getAbsPosition();
    inputs.heliumRelativeRotations =
        pivotEncoder.getPosition(); // logged for checking if postion as been initialized

    if (Constants.outtakeTuningMode) {
      inputs.debugTargetRPS = outtakeFlywheelSpeed.getDouble(0.0);
      inputs.targetPivotPosition = pivotPosition.getDouble(0.0);
    }

    heliumAbsoluteRotations = inputs.heliumAbsRotations;
  }

  @Override
  public void setOuttakeRPS(double desiredTopVelocityRPS, double desiredBottomVelocityRPS) {
    rightOuttakeMotor.setControl(new VelocityVoltage(desiredBottomVelocityRPS));
  }

  @Override
  public boolean initPivot() {
    if (heliumAbsoluteRotations
        > Constants.EncoderInitializeConstants.absEncoderMaxZeroingThreshold) {
      // Assume that abs position higher than maxValue is below the
      // hard stop zero point of shooter/deployer
      // If so, assume that position is 0 for motor internal encoder
      pivotMotor.setPosition(0);
    } else {
      pivotMotor.setPosition(
          heliumAbsoluteRotations * OuttakeConstants.gearReductionEncoderToMotor);
    }

    if (OrangeMath.equalToTwoDecimal(pivotEncoder.getVelocity(), 0)) {
      // Set only relative encoder rotations of Helium encoder to a very high number after
      // initialized
      // once
      // Relative encoder on Helium used only to check if we have already initialized after power
      // cycles
      pivotEncoder.setPosition(Constants.EncoderInitializeConstants.initializedRotationsFlag);
      return true;
    }
    return false;
  }

  @Override
  public void setPivotTarget(double rotations) {
    pivotMotor.setControl(new PositionVoltage(rotations));
  }

  @Override
  public void setPivotBrakeMode() {
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    Logger.recordOutput("Outtake/Hardware/PivotNeutralMode", "Brake");
  }

  @Override
  public void setPivotCoastMode() {
    pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    Logger.recordOutput("Outtake/Hardware/PivotNeutralMode", "Coast");
  }

  @Override
  public void stopOuttake() {
    rightOuttakeMotor.stopMotor();
  }

  @Override
  public void stopPivot() {
    pivotMotor.stopMotor();
  }
}
