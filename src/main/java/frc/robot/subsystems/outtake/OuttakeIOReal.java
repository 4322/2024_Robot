package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
  private TalonFX topOuttakeMotor;
  private TalonFX bottomOuttakeMotor;
  private TalonFX pivotMotor;
  private Canandcoder pivotEncoder;
  // shuffleboard
  ShuffleboardTab tab;
  GenericEntry outtakeFlywheelSpeed;
  GenericEntry pivotPosition;

  public OuttakeIOReal() {
    topOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.leftOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    bottomOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.rightOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    pivotMotor =
        new TalonFX(OuttakeConstants.pivotDeviceID, Constants.DriveConstants.Drive.canivoreName);
    pivotEncoder = new Canandcoder(OuttakeConstants.pivotEncoderID);

    configOuttake(topOuttakeMotor);
    configOuttake(bottomOuttakeMotor);
    configPivot(pivotMotor);
    if (Constants.debug) {
      tab = Shuffleboard.getTab("Outtake");
      outtakeFlywheelSpeed =
          tab.add("Desired Flywheel Velocity (RPS)", 0)
              .withSize(1, 1)
              .withPosition(0, 0)
              .getEntry();
      pivotPosition =
          tab.add("Pivot Position (Rotations)", 0).withSize(1, 1).withPosition(1, 0).getEntry();
    }
  }

  private void configOuttake(TalonFX talon) {
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
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
    currentLimitsConfigs.StatorCurrentLimitEnable = Constants.OuttakeConstants.statorEnabled;
    currentLimitsConfigs.StatorCurrentLimit = Constants.OuttakeConstants.shooterStatorLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = Constants.OuttakeConstants.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.OuttakeConstants.shooterSupplyLimit;

    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    talon.getConfigurator().apply(hardwareLimitSwitchConfigs);
    talon.getConfigurator().apply(currentLimitsConfigs);
    talon.getConfigurator().apply(slot0Configs);
    talon.getConfigurator().apply(closedLoopRampsConfigs);
    talon.getConfigurator().apply(openLoopRampsConfigs);
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
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = OuttakeConstants.pivotClosedLoopSec;
    motorOutputConfigs.NeutralMode = OuttakeConstants.pivotDefaultNeutralMode;
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
    
    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    talon.getConfigurator().apply(hardwareLimitSwitchConfigs);
    talon.getConfigurator().apply(currentLimitsConfigs);
    talon.getConfigurator().apply(slot0Configs);
    talon.getConfigurator().apply(closedLoopRampsConfigs);
    talon.getConfigurator().apply(voltageConfigs);
    talon.getConfigurator().apply(motorOutputConfigs);
    talon.getConfigurator().apply(softwareLimitSwitchConfigs);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.topCurrentAmps = topOuttakeMotor.getSupplyCurrent().getValue();
    inputs.topTempC = topOuttakeMotor.getDeviceTemp().getValue();
    inputs.topRotationsPerSec = topOuttakeMotor.getVelocity().getValue();

    inputs.bottomCurrentAmps = bottomOuttakeMotor.getSupplyCurrent().getValue();
    inputs.bottomTempC = bottomOuttakeMotor.getDeviceTemp().getValue();
    inputs.bottomRotationsPerSec = bottomOuttakeMotor.getVelocity().getValue();
    inputs.bottomOuttakeIsAlive = bottomOuttakeMotor.isAlive();
    inputs.topOuttakeIsAlive = topOuttakeMotor.isAlive();

    inputs.pivotRotations = pivotMotor.getPosition().getValue();
    inputs.pivotRotationsPerSec = pivotMotor.getVelocity().getValue() / 60;
    inputs.pivotAppliedVolts =
        pivotMotor.getDutyCycle().getValue() / 2 * pivotMotor.getSupplyVoltage().getValue();
    inputs.pivotCurrentAmps = pivotMotor.getSupplyCurrent().getValue();
    inputs.pivotTempC = pivotMotor.getDeviceTemp().getValue();
    inputs.pivotIsAlive = pivotMotor.isAlive();

    inputs.heliumAbsRotations = pivotEncoder.getAbsPosition();
    inputs.heliumRelativeRotations =
        pivotEncoder.getPosition(); // logged for checking if postion as been initialized

    if (Constants.debug) {
      inputs.debugTargetRPS = outtakeFlywheelSpeed.getDouble(0);
      inputs.targetPivotPosition = pivotPosition.getDouble(0);
    }
  }

  @Override
  public void setOuttakeRPS(double desiredTopVelocityRPS, double desiredBottomVelocityRPS) {
    topOuttakeMotor.setControl(new VelocityVoltage(desiredTopVelocityRPS));
    bottomOuttakeMotor.setControl(new VelocityVoltage(desiredBottomVelocityRPS));
  }

  @Override
  public boolean initPivot() {
    pivotMotor.setPosition(
        pivotEncoder.getAbsPosition() * OuttakeConstants.gearReductionEncoderToMotor);
    // set only relative encoder rotations of Helium encoder to a very high number after initialized
    // once
    // relative encoder on Helium used only to check if we have already initialized after power
    // cycle
    pivotEncoder.setPosition(Constants.EncoderInitializeConstants.setRelativeRotations);
    return OrangeMath.equalToTwoDecimal(pivotEncoder.getVelocity(), 0);
  }

  @Override
  public void setPivotTarget(double rotations) {
    pivotMotor.setControl(new PositionVoltage(rotations));
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
    mOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    topOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    bottomOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    pivotMotor.getConfigurator().refresh(mOutputConfigs);
    Logger.recordOutput("Outtake/Hardware/NeutralMode", "Brake");
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
    mOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    topOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    bottomOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    pivotMotor.getConfigurator().refresh(mOutputConfigs);
    Logger.recordOutput("Outtake/Hardware/NeutralMode", "Coast");
  }

  @Override
  public void stopOuttake() {
    topOuttakeMotor.stopMotor();
    bottomOuttakeMotor.stopMotor();
  }

  @Override
  public void stopPivot() {
    pivotMotor.stopMotor();
  }
}
