package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
  GenericEntry tuneOuttakeOverrideEnable;

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
    if (Constants.debug) {
      tab = Shuffleboard.getTab("Outtake");

      outtakeFlywheelSpeed =
          tab.add("Desired Flywheel Velocity (RPS)", 0)
              .withSize(1, 1)
              .withPosition(0, 0)
              .getEntry();
      pivotPosition =
          tab.add("Pivot Position (Rotations)", 0).withSize(1, 1).withPosition(1, 0).getEntry();
      if (Constants.outtakeTuningMode) {
        tuneOuttakeOverrideEnable =
            tab.add("debugOverride", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withSize(1, 1)
                .withPosition(1, 1)
                .getEntry();
      }
    }
  }

  private void configOuttake(TalonFX talon) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.Slot0.kP = Constants.OuttakeConstants.kP;
    config.Slot0.kI = Constants.OuttakeConstants.kI;
    config.Slot0.kD = Constants.OuttakeConstants.kD;
    config.Slot0.kV = Constants.OuttakeConstants.kV;
    config.Slot0.kS = Constants.OuttakeConstants.kS;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.OuttakeConstants.closedLoopRampSec;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.OuttakeConstants.openLoopRampSec;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.OuttakeConstants.shooterStatorLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.OuttakeConstants.shooterSupplyLimit;
    config.CurrentLimits.SupplyCurrentThreshold =
        Constants.OuttakeConstants.shooterSupplyCurrentThreshold;
    config.CurrentLimits.SupplyTimeThreshold =
        Constants.OuttakeConstants.shooterSupplyTimeThreshold;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    talon.getConfigurator().apply(config);
  }

  private void configPivot(TalonFX talon) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = Constants.OuttakeConstants.pivotkP;
    config.Slot0.kI = OuttakeConstants.pivotkI;
    config.Slot0.kD = OuttakeConstants.pivotkD;
    config.Voltage.PeakForwardVoltage = OuttakeConstants.pivotPeakForwardVoltage;
    config.Voltage.PeakReverseVoltage = OuttakeConstants.pivotPeakReverseVoltage;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = OuttakeConstants.pivotClosedLoopSec;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = OuttakeConstants.limitForwardMotion;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = OuttakeConstants.limitReverseMotion;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        OuttakeConstants.forwardSoftLimitThresholdRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        OuttakeConstants.reverseSoftLimitThresholdRotations;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.OuttakeConstants.pivotStatorLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.OuttakeConstants.pivotSupplyLimit;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    talon.getConfigurator().apply(config);

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

    inputs.rightSupplyCurrentAmps = rightOuttakeMotor.getSupplyCurrent().getValue();
    inputs.rightStatorCurrentAmps = rightOuttakeMotor.getStatorCurrent().getValue();
    inputs.rightTempC = rightOuttakeMotor.getDeviceTemp().getValue();
    inputs.rightRotationsPerSec = rightOuttakeMotor.getVelocity().getValue();
    inputs.rightOuttakeIsAlive = rightOuttakeMotor.isAlive();

    inputs.pivotRotations = pivotMotor.getPosition().getValue();
    inputs.pivotRotationsPerSec = pivotMotor.getVelocity().getValue();
    inputs.pivotAppliedVolts =
        pivotMotor.getDutyCycle().getValue() / 2 * pivotMotor.getSupplyVoltage().getValue();
    inputs.pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent().getValue();
    inputs.pivotStatorCurrentAmps = pivotMotor.getStatorCurrent().getValue();
    inputs.pivotTempC = pivotMotor.getDeviceTemp().getValue();
    inputs.pivotIsAlive = pivotMotor.isAlive();

    inputs.heliumAbsRotations = pivotEncoder.getAbsPosition();
    inputs.heliumRelativeRotations =
        pivotEncoder.getPosition(); // logged for checking if postion as been initialized

    if (Constants.outtakeTuningMode) {
      inputs.debugTargetRPS = outtakeFlywheelSpeed.getDouble(0);
      inputs.targetPivotPosition = pivotPosition.getDouble(0);
      inputs.tuneOuttakeOverrideEnable = tuneOuttakeOverrideEnable.getBoolean(false);
    }
    if (inputs.heliumAbsRotations
        > Constants.EncoderInitializeConstants.absEncoderMaxZeroingThreshold) {
      inputs.heliumAbsRotations = 0;
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

    if (OrangeMath.equalToEpsilon(pivotEncoder.getVelocity(), 0.0, 0.1)) {
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
