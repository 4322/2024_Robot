package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import org.littletonrobotics.junction.Logger;

public class OuttakeIOReal implements OuttakeIO {
  private TalonFX topOuttakeMotor;
  private TalonFX bottomOuttakeMotor;
  private TalonFX pivotMotor;
  private Canandcoder pivotEncoder;
  // shuffleboard
  ShuffleboardTab tab;
  GenericEntry topOuttakeFlywheelSpeed;
  GenericEntry bottomOuttakeFlywheelSpeed;
  GenericEntry pivotPosition;
  GenericEntry tuneOuttakeOverrideEnable;

  private boolean initialized;

  public OuttakeIOReal() {
    topOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.topOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    bottomOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.bottomOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    pivotMotor =
        new TalonFX(OuttakeConstants.pivotDeviceID, Constants.DriveConstants.Drive.canivoreName);
    pivotEncoder = new Canandcoder(OuttakeConstants.pivotEncoderID);

    configOuttake(topOuttakeMotor, true);
    configOuttake(bottomOuttakeMotor, false);
    configPivot(pivotMotor);
    if (Constants.debug) {
      tab = Shuffleboard.getTab("Outtake");

      topOuttakeFlywheelSpeed =
          tab.add("Top Desired Flywheel Velocity (RPS)", 0)
              .withSize(1, 1)
              .withPosition(0, 0)
              .getEntry();
      bottomOuttakeFlywheelSpeed = 
          tab.add("Bottom Desired Flywheel Velocity (RPS)", 0)
              .withSize(1, 1)
              .withPosition(2, 0)
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

  private void configOuttake(TalonFX talon, boolean isTopOuttakeMotor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    if (isTopOuttakeMotor) {
      config.Slot0.kP = Constants.OuttakeConstants.topkP;
      config.Slot0.kI = Constants.OuttakeConstants.topkI;
      config.Slot0.kD = Constants.OuttakeConstants.topkD;
      config.Slot0.kV = Constants.OuttakeConstants.topkV;
      config.Slot0.kS = Constants.OuttakeConstants.topkS;
    }
    else {
      config.Slot0.kP = Constants.OuttakeConstants.bottomkP;
      config.Slot0.kI = Constants.OuttakeConstants.bottomkI;
      config.Slot0.kD = Constants.OuttakeConstants.bottomkD;
      config.Slot0.kV = Constants.OuttakeConstants.bottomkV;
      config.Slot0.kS = Constants.OuttakeConstants.bottomkS;
    }

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
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = OuttakeConstants.limitReverseMotion;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        OuttakeConstants.reverseSoftLimitThresholdRotations;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.OuttakeConstants.pivotStatorLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.OuttakeConstants.pivotSupplyLimit;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    talon.getConfigurator().apply(config);

    // zero helium abs encoder on the stop bar
    // 60 degree shooting angle abs position is about 0.72 rotations
    // the encoder wraps at about 78 degrees
    Canandcoder.Settings settings = new Canandcoder.Settings();
    settings.setInvertDirection(false);
    settings.setPositionFramePeriod(0.010);
    settings.setVelocityFramePeriod(0.050);
    settings.setStatusFramePeriod(1.0);
    pivotEncoder.setSettings(settings, 0.050);

    try {
      Thread.sleep(50); // 5 status frames to be safe
    } catch (InterruptedException e) {
    }

    double currentPivotPosition = pivotEncoder.getAbsPosition();
      // The abs encoder position must not be within the specified flag range.
      // The specified range assumes that the shooter pivot is too far below 
      // the zero point and is wrapping around to 1 rotation.
    if (!(currentPivotPosition > Constants.OuttakeConstants.absEncoderMaxZeroingThreshold
          && currentPivotPosition < Constants.OuttakeConstants.absEncoderAlmostZeroThreshold)) {
      // If abs encoder is close to 1 rotation, it means that pivot is just a bit below zero point 
      // and therefore should we should just treat it as zero
      if (currentPivotPosition < 1 && currentPivotPosition > Constants.OuttakeConstants.absEncoderAlmostZeroThreshold) {
        currentPivotPosition = 0;
      }
      pivotMotor.setPosition(currentPivotPosition * OuttakeConstants.gearReductionEncoderToMotor);
      DriverStation.reportWarning("Initialized shooter pivot", false);
      initialized = true;
    }
    else {
      DriverStation.reportError("Failed to initialize shooter pivot at " + currentPivotPosition + " helium rotations", false);
    }
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.topCurrentAmps = topOuttakeMotor.getSupplyCurrent().getValue();
    inputs.topTempC = topOuttakeMotor.getDeviceTemp().getValue();
    inputs.topRotationsPerSec = topOuttakeMotor.getVelocity().getValue();
    inputs.topOuttakeIsAlive = topOuttakeMotor.isAlive();

    inputs.bottomSupplyCurrentAmps = bottomOuttakeMotor.getSupplyCurrent().getValue();
    inputs.bottomStatorCurrentAmps = bottomOuttakeMotor.getStatorCurrent().getValue();
    inputs.bottomTempC = bottomOuttakeMotor.getDeviceTemp().getValue();
    inputs.bottomRotationsPerSec = bottomOuttakeMotor.getVelocity().getValue();
    inputs.bottomOuttakeIsAlive = bottomOuttakeMotor.isAlive();

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
      inputs.topDebugTargetRPS = topOuttakeFlywheelSpeed.getDouble(0);
      inputs.bottomDebugTargetRPS = bottomOuttakeFlywheelSpeed.getDouble(0);
      inputs.targetPivotPosition = pivotPosition.getDouble(0);
      inputs.tuneOuttakeOverrideEnable = tuneOuttakeOverrideEnable.getBoolean(false);
    }
  }

  @Override
  public void setOuttakeRPS(double desiredTopVelocityRPS, double desiredBottomVelocityRPS) {
    bottomOuttakeMotor.setControl(new VelocityVoltage(-desiredBottomVelocityRPS).withEnableFOC(false));
    topOuttakeMotor.setControl(new VelocityVoltage(desiredTopVelocityRPS).withEnableFOC(false));
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
  public void setFlywheelBrakeMode() {
    topOuttakeMotor.setNeutralMode(NeutralModeValue.Brake);
    bottomOuttakeMotor.setNeutralMode(NeutralModeValue.Brake);
    Logger.recordOutput("Outtake/Hardware/FlywheelNeutralMode", "Brake");
  }

  @Override
  public void setFlywheelCoastMode() {
    topOuttakeMotor.setNeutralMode(NeutralModeValue.Coast);
    bottomOuttakeMotor.setNeutralMode(NeutralModeValue.Coast);
    Logger.recordOutput("Outtake/Hardware/FlywheelNeutralMode", "Coast");
  }

  @Override
  public void stopOuttake() {
    bottomOuttakeMotor.stopMotor();
    topOuttakeMotor.stopMotor();
  }

  @Override
  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  @Override
  public boolean pivotIsInitialized() {
    return initialized;
  }
}
