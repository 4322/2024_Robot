package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
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
  private double currentWheelDegrees;
  private double currentMotorRotations;

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
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveConstants.Drive.closedLoopRampSec;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = DriveConstants.Drive.openLoopRampSec;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = DriveConstants.Drive.statorLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = DriveConstants.Drive.supplyLimit;
    config.CurrentLimits.SupplyCurrentThreshold = DriveConstants.Drive.supplyThreshold;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
    // Without this code, all bevel gears would need to face right when the modules are zeroed.
    boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
    if (isLeftSide) {
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    talonFX.getConfigurator().apply(config);

    // need rapid velocity feedback for control logic
    talonFX
        .getVelocity()
        .setUpdateFrequency(
            OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()),
            Constants.controllerConfigTimeoutMs);

    // need rapid position feedback for accurate odometry
    talonFX
        .getPosition()
        .setUpdateFrequency(
            OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()),
            Constants.controllerConfigTimeoutMs);
  }

  private void configRotation(TalonFX talonFX, WheelPosition wheelPos) {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    motorConfig.Slot0.kP = robotSpecificConstants.getRotationkP();
    motorConfig.Slot0.kD = robotSpecificConstants.getRotationkD();
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        DriveConstants.Rotation.configCLosedLoopRamp;
    motorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    motorConfig.Voltage.PeakForwardVoltage = DriveConstants.Rotation.maxPower;
    motorConfig.Voltage.PeakReverseVoltage = -DriveConstants.Rotation.maxPower;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = Constants.DriveConstants.Rotation.statorLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.Rotation.supplyLimit;

    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    StatusCode encoderConfigStatus = encoder.getConfigurator().apply(encoderConfig);
    StatusCode motorConfigStatus = talonFX.getConfigurator().apply(motorConfig);

    if (encoderConfigStatus != StatusCode.OK) {
      DriverStation.reportError("CANCoder " + encoder.getDeviceID() + " error: " + motorConfigStatus.getDescription(), false);
    }
    if (motorConfigStatus != StatusCode.OK) {
      DriverStation.reportError("Talon " + talonFX.getDeviceID() + " error: " + motorConfigStatus.getDescription(), false);
    }

    // need fast initial reading from the CANCoder
    encoder
        .getPosition()
        .setUpdateFrequency(OrangeMath.msAndHzConverter(10), Constants.controllerConfigTimeoutMs);
    try {
      Thread.sleep(50); // 5 status frames to be safe
    } catch (InterruptedException e) {
    }

    // initialize internal Falcon encoder to absolute wheel position from CANCoder
    double count =
        (encoder.getAbsolutePosition().getValueAsDouble()
                - DriveConstants.Rotation.CANCoderOffsetRotations[wheelPos.wheelNumber])
            * robotSpecificConstants.getRotationGearRatio();
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

    // only need the CANcoder for AdvantageKit logging after this
    encoder
        .getPosition()
        .setUpdateFrequency(OrangeMath.msAndHzConverter(200), Constants.controllerConfigTimeoutMs);
  }

  // Below are the implementations of the methods in SwerveModuleIO.java
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // drive inputs
    inputs.driveMeters =
        driveMotor.getPosition().getValue()
            * Math.PI
            * OrangeMath.inchesToMeters(Constants.DriveConstants.Drive.wheelDiameterInches)
            / robotSpecificConstants.getDriveGearRatio();
    inputs.driveMetersPerSec =
        driveMotor.getVelocity().getValue()
            * Math.PI
            * OrangeMath.inchesToMeters(Constants.DriveConstants.Drive.wheelDiameterInches)
            / robotSpecificConstants.getDriveGearRatio();
    inputs.driveAppliedVolts =
        driveMotor.getDutyCycle().getValue() / 2 * driveMotor.getSupplyVoltage().getValue();
    inputs.driveSupplyCurrentAmps = driveMotor.getSupplyCurrent().getValue();
    inputs.driveStatorCurrentAmps = driveMotor.getStatorCurrent().getValue();
    // turn inputs
    inputs.turnVelocityDegPerSec = Units.rotationsToDegrees(turningMotor.getVelocity().getValue());
    inputs.turnAppliedVolts =
        turningMotor.getDutyCycle().getValue() / 2 * turningMotor.getSupplyVoltage().getValue();
    inputs.turnSupplyCurrentAmps = turningMotor.getSupplyCurrent().getValue();
    inputs.turnStatorCurrentAmps = turningMotor.getStatorCurrent().getValue();
    inputs.turnDegrees = Units.rotationsToDegrees(turningMotor.getPosition().getValue());
    inputs.turnRotations = turningMotor.getPosition().getValue();
    inputs.wheelDegreesTo360 =
        MathUtil.inputModulus(
            inputs.turnDegrees / robotSpecificConstants.getRotationGearRatio(), 0, 360);

    inputs.calculatedFF = calcFeedForwardVoltsOverMetersPerSec;
    inputs.calculatedVolts = desiredVolts;

    inputs.absEncoderRotations = encoder.getAbsolutePosition().getValueAsDouble();

    currentMotorRotations = inputs.turnRotations;
    currentWheelDegrees = inputs.wheelDegreesTo360;
  }

  // PID methods for turn motor
  @Override
  public void setTurnAngle(double desiredAngle) {
    // Calculates change in degrees and adds to current position after converting to encoder
    // rotations
    turningMotor.setControl(
        new PositionVoltage(
            currentMotorRotations
                + (OrangeMath.boundDegrees(desiredAngle - currentWheelDegrees))
                    / 360.0
                    * robotSpecificConstants.getRotationGearRatio()));
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
    driveMotor.setControl(new VoltageOut(desiredVolts).withEnableFOC(true));
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
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turningMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCoastMode() {
    driveMotor.setNeutralMode(NeutralModeValue.Coast);
    turningMotor.setNeutralMode(NeutralModeValue.Coast);
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
