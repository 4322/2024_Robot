package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.robot.RobotChooser.RobotChooser;
import frc.robot.RobotChooser.RobotChooserInterface;
import frc.utility.CanBusUtil;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
  private RobotChooserInterface robotSpecificConstants = RobotChooser.getInstance().getConstants();

  private CANSparkMax driveMotor;

  private CANSparkMax turningMotor;
  private SparkAbsoluteEncoder encoder;

  private double[] feedForwardMetersPerSecThreshold =
      robotSpecificConstants.getDriveffSpeedMetersPerSecThresholds().clone();
  private double[] feedForwardVoltsOverMetersPerSec =
      robotSpecificConstants.getDriveffVoltsOverMetersPerSec().clone();
  private double kSVolts = robotSpecificConstants.getDrivekSVolts();

  private double calcFeedForwardVoltsOverMetersPerSec;
  private double desiredVolts;

  public SwerveModuleIOSparkMax(WheelPosition wheelPos) {
    switch (wheelPos) {
      case FRONT_RIGHT:
        driveMotor =
            new CANSparkMax(robotSpecificConstants.getFrontRightDriveID(), MotorType.kBrushless);
        turningMotor =
            new CANSparkMax(robotSpecificConstants.getFrontRightRotationID(), MotorType.kBrushless);
        break;
      case FRONT_LEFT:
        driveMotor =
            new CANSparkMax(robotSpecificConstants.getFrontLeftDriveID(), MotorType.kBrushless);
        turningMotor =
            new CANSparkMax(robotSpecificConstants.getFrontLeftRotationID(), MotorType.kBrushless);
        break;
      case BACK_RIGHT:
        driveMotor =
            new CANSparkMax(robotSpecificConstants.getBackRightDriveID(), MotorType.kBrushless);
        turningMotor =
            new CANSparkMax(robotSpecificConstants.getBackRightRotationID(), MotorType.kBrushless);
        break;
      case BACK_LEFT:
        driveMotor =
            new CANSparkMax(robotSpecificConstants.getBackLeftDriveID(), MotorType.kBrushless);
        turningMotor =
            new CANSparkMax(robotSpecificConstants.getBackLeftRotationID(), MotorType.kBrushless);
        break;
    }

    encoder = turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    CanBusUtil.staggerSparkMax(turningMotor);
    CanBusUtil.staggerSparkMax(driveMotor);

    configDrive(driveMotor, wheelPos);

    configRotation(turningMotor);
  }

  private void configDrive(CANSparkMax sparkMax, WheelPosition pos) {
    sparkMax.restoreFactoryDefaults();

    sparkMax.setClosedLoopRampRate(DriveConstants.Drive.closedLoopRampSec);
    sparkMax.setOpenLoopRampRate(DriveConstants.Drive.openLoopRampSec);
    sparkMax.setSmartCurrentLimit(DriveConstants.Drive.currentLimit);
    sparkMax.setSecondaryCurrentLimit(DriveConstants.Drive.secondaryCurrentLimit);
    sparkMax.enableVoltageCompensation(DriveConstants.Drive.voltageCompSaturation);
    sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling
    sparkMax
        .getEncoder()
        .setPositionConversionFactor(
            Math.PI
                * Constants.DriveConstants.Drive.wheelDiameterInches
                * Constants.inchesToMeters
                / robotSpecificConstants.getDriveGearRatio()); // motor rotations to wheel meters
    sparkMax
        .getEncoder()
        .setVelocityConversionFactor(
            Math.PI
                * Constants.DriveConstants.Drive.wheelDiameterInches
                * Constants.inchesToMeters
                / 60
                / robotSpecificConstants.getDriveGearRatio()); // motor RPM to wheel m/s

    // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
    // Without this code, all bevel gears would need to face right when the modules are zeroed.
    boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
    sparkMax.setInverted(isLeftSide);

    // need rapid velocity feedback for control logic and position for odometry
    CanBusUtil.fastVelocityPositionSparkMax(driveMotor);
  }

  private void configRotation(CANSparkMax sparkMax) {
    sparkMax.restoreFactoryDefaults();

    encoder.setInverted(true);
    turningMotor.setInverted(true);

    SparkPIDController config = sparkMax.getPIDController();
    config.setP(robotSpecificConstants.getRotationkP(), 0);
    config.setD(robotSpecificConstants.getRotationkD(), 0);
    sparkMax.setClosedLoopRampRate(DriveConstants.Rotation.configCLosedLoopRamp);
    config.setSmartMotionAllowedClosedLoopError(
        DriveConstants.Rotation.allowableClosedloopError, 0);
    config.setOutputRange(-DriveConstants.Rotation.maxPower, DriveConstants.Rotation.maxPower);
    sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling

    sparkMax.enableVoltageCompensation(DriveConstants.Rotation.configVoltageCompSaturation);
    sparkMax.setSmartCurrentLimit(
        DriveConstants.Rotation.stallLimit, DriveConstants.Rotation.freeLimit);
    encoder.setPositionConversionFactor(360); // convert encoder position duty cycle to degrees
    config.setFeedbackDevice(encoder);
    config.setPositionPIDWrappingEnabled(true);
    config.setPositionPIDWrappingMinInput(0);
    config.setPositionPIDWrappingMaxInput(360);

    // need rapid position feedback for steering control
    CanBusUtil.fastPositionSparkMaxAbs(turningMotor);
  }

  // Below are the implementations of the methods in SwerveModuleIO.java
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // drive inputs
    inputs.driveMeters = driveMotor.getEncoder().getPosition();
    inputs.driveMetersPerSec = driveMotor.getEncoder().getVelocity();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveSupplyCurrentAmps = driveMotor.getOutputCurrent();

    // turn inputs
    inputs.turnVelocityDegPerSec = Units.rotationsToDegrees(encoder.getVelocity());
    inputs.turnAppliedVolts = turningMotor.getAppliedOutput() * turningMotor.getBusVoltage();
    inputs.turnSupplyCurrentAmps = turningMotor.getOutputCurrent();
    inputs.turnDegrees = encoder.getPosition();

    inputs.calculatedFF = calcFeedForwardVoltsOverMetersPerSec;
    inputs.calculatedVolts = desiredVolts;
  }

  // set turn angle of wheel using hardware PID
  @Override
  public void setTurnAngle(double desiredAngle) {
    turningMotor.getPIDController().setReference(desiredAngle, ControlType.kPosition);
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
              / (feedForwardMetersPerSecThreshold[lastElement]
                  - feedForwardMetersPerSecThreshold[
                      secondToLastElement]); // TODO: fix potential divide by 0

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
              / (feedForwardMetersPerSecThreshold[lowerBound]
                  - feedForwardMetersPerSecThreshold[
                      upperBound]); // TODO: fix potential divide by 0

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
    REVLibError error =
        driveMotor.getPIDController().setReference(desiredVolts, ControlType.kVoltage, 0);
    if (error != REVLibError.kOk) {
      DriverStation.reportError(
          "Drive motor "
              + driveMotor.getDeviceId()
              + " error "
              + error.name()
              + " while sending requested voltage",
          false);
    }
  }

  @Override
  public void setFeedForwardSpeedThreshold(double[] newFeedForwardMetersPerSecThreshold) {
    for (int i = 0; i < feedForwardMetersPerSecThreshold.length; i++) {
      feedForwardMetersPerSecThreshold[i] = newFeedForwardMetersPerSecThreshold[i];
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
    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void setCoastMode() {
    driveMotor.setIdleMode(IdleMode.kCoast);
    turningMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void setClosedRampRate(double period) {
    driveMotor.setClosedLoopRampRate(period);
  }

  @Override
  public void setOpenRampRate(double period) {
    driveMotor.setOpenLoopRampRate(period);
  }

  @Override
  public void stopMotor() {
    driveMotor.stopMotor();
    turningMotor.stopMotor();
    // for logging purposes
    desiredVolts = 0;
    calcFeedForwardVoltsOverMetersPerSec = 0;
  }
}
