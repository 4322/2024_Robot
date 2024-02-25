package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.robot.RobotChooser.RobotChooser;
import frc.robot.RobotChooser.RobotChooserInterface;
import frc.utility.CanBusUtil;
import frc.utility.OrangeMath;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  private RobotChooserInterface robotSpecificConstants = RobotChooser.getInstance().getConstants();
  private TalonFX driveMotor;

  private TalonFX turningMotor;
  private CANcoder encoder;

  private double[] feedForwardMetersPerSecThreshold =
      robotSpecificConstants.getDriveffSpeedMetersPerSecThresholds().clone();
  private double[] feedForwardVoltsOverMetersPerSec =
      robotSpecificConstants.getDriveffVoltsOverMetersPerSec().clone();
  private double kSVolts = robotSpecificConstants.getDrivekSVolts();

  private double calcFeedForwardVoltsOverMetersPerSec;
  private double desiredVolts;

  public SwerveModuleIOTalonFX(WheelPosition wheelPos) {
    switch (wheelPos) {
      case FRONT_RIGHT:
        driveMotor =
            new TalonFX(
                robotSpecificConstants.getFrontRightDriveID(),
                Constants.DriveConstants.Drive.canivoreName);
        turningMotor =
            new TalonFX(
                robotSpecificConstants.getFrontRightRotationID(),
                Constants.DriveConstants.Drive.canivoreName);
        encoder =
            new CANcoder(
                DriveConstants.Drive.frontRightEncoderID,
                Constants.DriveConstants.Drive.canivoreName);
        break;
      case FRONT_LEFT:
        driveMotor =
            new TalonFX(
                robotSpecificConstants.getFrontLeftDriveID(),
                Constants.DriveConstants.Drive.canivoreName);
        turningMotor =
            new TalonFX(
                robotSpecificConstants.getFrontLeftRotationID(),
                Constants.DriveConstants.Drive.canivoreName);
        encoder =
            new CANcoder(
                DriveConstants.Drive.frontLeftEncoderID,
                Constants.DriveConstants.Drive.canivoreName);
        break;
      case BACK_RIGHT:
        driveMotor =
            new TalonFX(
                robotSpecificConstants.getBackRightDriveID(),
                Constants.DriveConstants.Drive.canivoreName);
        turningMotor =
            new TalonFX(
                robotSpecificConstants.getBackRightRotationID(),
                Constants.DriveConstants.Drive.canivoreName);
        encoder =
            new CANcoder(
                DriveConstants.Drive.rearRightEncoderID,
                Constants.DriveConstants.Drive.canivoreName);
        break;
      case BACK_LEFT:
        driveMotor =
            new TalonFX(
                robotSpecificConstants.getBackLeftDriveID(),
                Constants.DriveConstants.Drive.canivoreName);
        turningMotor =
            new TalonFX(
                robotSpecificConstants.getBackLeftRotationID(),
                Constants.DriveConstants.Drive.canivoreName);
        encoder =
            new CANcoder(
                DriveConstants.Drive.rearLeftEncoderID,
                Constants.DriveConstants.Drive.canivoreName);
        break;
    }

    configDrive(driveMotor, wheelPos);

    configRotation(turningMotor, wheelPos);
  }

  private void configDrive(TalonFX talonFX, WheelPosition pos) {
    talonFX.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfigurator config = talonFX.getConfigurator();
    Slot0Configs slot0Configs = new Slot0Configs();
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = DriveConstants.Drive.closedLoopRampSec;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = DriveConstants.Drive.openLoopRampSec;
    currentLimitsConfigs.StatorCurrentLimit = DriveConstants.Drive.statorLimit;
    currentLimitsConfigs.SupplyCurrentLimit = DriveConstants.Drive.supplyLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = DriveConstants.Drive.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentThreshold = DriveConstants.Drive.supplyThreshold;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    config.apply(slot0Configs);
    config.apply(closedLoopRampsConfigs);
    config.apply(openLoopRampsConfigs);
    config.apply(currentLimitsConfigs);
    config.apply(motorOutputConfigs);
    // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
    // Without this code, all bevel gears would need to face right when the modules are zeroed.
    boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
    talonFX.setInverted(isLeftSide);

    // need rapid velocity feedback for control logic
    talonFX
        .getVelocity()
        .setUpdateFrequency(
            OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()),
            Constants.controllerConfigTimeoutMs);
  }

  private void configRotation(TalonFX talonFX, WheelPosition wheelPos) {
    talonFX.getConfigurator().apply(new TalonFXConfiguration());

    Slot0Configs slot0Configs = new Slot0Configs();
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();

    talonFX.setInverted(true);

    slot0Configs.kP = robotSpecificConstants.getRotationkP();
    slot0Configs.kD = robotSpecificConstants.getRotationkD();
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        DriveConstants.Rotation.configCLosedLoopRamp;
    closedLoopGeneralConfigs.ContinuousWrap = false;
    voltageConfigs.PeakForwardVoltage = DriveConstants.Rotation.maxPower;
    voltageConfigs.PeakReverseVoltage = -DriveConstants.Rotation.maxPower;
    motorOutputConfigs.NeutralMode =
        NeutralModeValue.Brake;

    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = false;

    canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoder.getConfigurator().apply(canConfig);
    talonFX.getConfigurator().apply(slot0Configs);
    talonFX.getConfigurator().apply(closedLoopRampsConfigs);
    talonFX.getConfigurator().apply(closedLoopGeneralConfigs);
    talonFX.getConfigurator().apply(voltageConfigs);
    talonFX.getConfigurator().apply(motorOutputConfigs);
    talonFX.getConfigurator().apply(hardwareLimitSwitchConfigs);
    // need fast initial reading from the CANCoder
    encoder
        .getPosition()
        .setUpdateFrequency(OrangeMath.msAndHzConverter(Constants.controllerConfigTimeoutMs));
    try {
      Thread.sleep(50); // 5 status frames to be safe
    } catch (InterruptedException e) {
    }

    // initialize internal Falcon encoder to absolute wheel position from CANCoder
    double count =
        (encoder.getAbsolutePosition().getValueAsDouble()
            - DriveConstants.Rotation.CANCoderOffsetRotations[wheelPos.wheelNumber]);
    StatusCode error = talonFX.setPosition(count, Constants.controllerConfigTimeoutMs);
    if (error != StatusCode.OK) {
      DriverStation.reportError(
          "Error " + error.value + " initializing Talon FX " + talonFX.getDeviceID() + " position ",
          false);
    }
    // need rapid position feedback for steering control
    talonFX
        .getPosition()
        .setUpdateFrequency(
            OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()),
            Constants.controllerConfigTimeoutMs);
  }

  // Below are the implementations of the methods in SwerveModuleIO.java
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // drive inputs
    inputs.driveMeters = driveMotor.getPosition().getValue();
    inputs.driveMetersPerSec = driveMotor.getVelocity().getValue() / 60;
    inputs.driveAppliedVolts =
        driveMotor.getDutyCycle().getValue() / 2 * driveMotor.getSupplyVoltage().getValue();
    inputs.driveCurrentAmps = driveMotor.getSupplyCurrent().getValue();
    // turn inputs
    inputs.turnVelocityDegPerSec = Units.rotationsToDegrees(turningMotor.getVelocity().getValue());
    inputs.turnAppliedVolts =
        turningMotor.getDutyCycle().getValue() / 2 * turningMotor.getSupplyVoltage().getValue();
    inputs.turnCurrentAmps = turningMotor.getSupplyCurrent().getValue();
    inputs.turnDegrees = Units.rotationsToDegrees(turningMotor.getPosition().getValue());

    inputs.calculatedFF = calcFeedForwardVoltsOverMetersPerSec;
    inputs.calculatedVolts = desiredVolts;

    inputs.absEncoderRotations = encoder.getAbsolutePosition().getValueAsDouble();
  }

  // PID methods for turn motor
  @Override
  public void setTurnAngle(double desiredAngle) {
    double currentRotPosition = turningMotor.getPosition().getValueAsDouble();
    double currentBoundedDegrees = OrangeMath.boundDegrees(currentRotPosition * 360);
    // Calculates change in degrees and adds to current position after converting to encoder rotations
    turningMotor.setControl(
        new PositionVoltage(currentRotPosition + (OrangeMath.boundDegrees(desiredAngle - currentBoundedDegrees) 
                              / 360.0 * robotSpecificConstants.getRotationGearRatio())));
  }

  // set drive motor voltage based on desired wheel m/s
  @Override
  public void setDriveVoltage(double desiredWheelMetersPerSec) {
    double desiredWheelMetersPerSecAbs = Math.abs(desiredWheelMetersPerSec);
    int i;

    // If wheel m/s less than element 1 of FF speed threshold, i defaults to 0
    // due to for loop iterating i at the end.
    // Greater than or equals to not used in order to protect against erroneous shuffleboard input
    // where element 0 of FF velocity threshold is changed from 0.
    for (i = feedForwardMetersPerSecThreshold.length - 1; i > 0; i--) {
      if (desiredWheelMetersPerSecAbs >= feedForwardMetersPerSecThreshold[i]) {
        break;
      }
    }

    // Linear extrapolation to account for edge case where requested wheel speed
    // is greater than max threshold speed.
    // Feed Forward calculated by determining linear equation between last two points.
    if (i == feedForwardMetersPerSecThreshold.length - 1) {
      int lastElement = i;
      int secondToLastElement = i - 1;
      // Slope between last point and second to last point used to predict corresponding
      // Feed Forward value for requested m/s values beyond max speed threshold
      double slope =
          (feedForwardVoltsOverMetersPerSec[lastElement]
                  - feedForwardVoltsOverMetersPerSec[secondToLastElement])
              / (Math.max(
                  feedForwardMetersPerSecThreshold[lastElement]
                      - feedForwardMetersPerSecThreshold[secondToLastElement],
                  0.2)); // TODO: fix potential divide by 0

      calcFeedForwardVoltsOverMetersPerSec =
          slope * (desiredWheelMetersPerSecAbs - feedForwardMetersPerSecThreshold[lastElement])
              + feedForwardVoltsOverMetersPerSec[lastElement];
    }
    // Linear interpolation to calculate a more precise Feed Forward value for
    // points between established thresholds.
    // Calculated through weighted average.
    else {
      int upperBound = i + 1;
      int lowerBound = i;
      // Calculated weight based on distance between lower bound value and desired wheel speed value
      double weight =
          (desiredWheelMetersPerSecAbs - feedForwardMetersPerSecThreshold[upperBound])
              / Math.min(
                  -0.2,
                  (feedForwardMetersPerSecThreshold[lowerBound]
                      - feedForwardMetersPerSecThreshold[
                          upperBound])); // TODO: fix potential divide by 0

      calcFeedForwardVoltsOverMetersPerSec =
          (weight * feedForwardVoltsOverMetersPerSec[lowerBound])
              + ((1 - weight) * feedForwardVoltsOverMetersPerSec[upperBound]);
    }

    // make sure wheel m/s shuffleboard inputs are in ascending order
    if (calcFeedForwardVoltsOverMetersPerSec < 0) {
      calcFeedForwardVoltsOverMetersPerSec = 0;
      kSVolts = 0;
    }

    // convert speed to volts while accounting for volts required to overcome static friction
    desiredVolts =
        Math.signum(desiredWheelMetersPerSec)
            * (kSVolts + (calcFeedForwardVoltsOverMetersPerSec * desiredWheelMetersPerSecAbs));

    // send requested voltage to SparkMAX
    driveMotor.setVoltage(desiredVolts);
  }

  @Override
  public void setFeedForwardSpeedThreshold(double[] newFeedForwardRPSThreshold) {
    for (int i = 0; i < feedForwardMetersPerSecThreshold.length; i++) {
      feedForwardMetersPerSecThreshold[i] = newFeedForwardRPSThreshold[i];
    }
  }

  @Override
  public void updateFeedForward(double[] newFeedForwardVolts) {
    for (int i = 0; i < feedForwardVoltsOverMetersPerSec.length; i++) {
      feedForwardVoltsOverMetersPerSec[i] = newFeedForwardVolts[i];
    }
  }

  @Override
  public void updateVoltsToOvercomeFriction(double newkSVolts) {
    kSVolts = newkSVolts;
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().refresh(motorOutputConfigs);
    turningMotor.getConfigurator().refresh(motorOutputConfigs);
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    driveMotor.getConfigurator().refresh(motorOutputConfigs);
    turningMotor.getConfigurator().refresh(motorOutputConfigs);
  }

  @Override
  public void setClosedRampRate(double period) {
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = period;
    driveMotor.getConfigurator().refresh(closedLoopRampsConfigs);
  }

  @Override
  public void setOpenRampRate(double period) {
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = period;
    driveMotor.getConfigurator().refresh(openLoopRampsConfigs);
  }

  @Override
  public void stopMotor() {
    driveMotor.stopMotor();
    turningMotor.stopMotor();
  }
}
