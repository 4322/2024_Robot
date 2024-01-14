// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utility.OrangeMath;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean debug = false;

  public static final boolean driveEnabled = true;
  public static final boolean gyroEnabled = true;
  public static final boolean joysticksEnabled = true;
  public static final boolean xboxEnabled = true;
  
  public static final boolean spinoutCenterEnabled = true;  // center rotate burst of power
  public static final boolean spinoutCornerEnabled = true;
  public static final boolean pseudoAutoRotateEnabled = false;
  public static final String driveInputScaling = InputScalingStrings.quadratic;
  public static final String controllerType = ControllerTypeStrings.xboxLeftDrive;
  
  public static final class InputScalingStrings{
    public static final String linear = "Linear";
    public static final String quadratic = "Quadratic";
    public static final String cubic = "Cubic";
  }

  public static final class ControllerTypeStrings{
    public static final String none = "None";
    public static final String xboxLeftDrive = "Xbox Left Drive";
    public static final String xboxRightDrive = "Xbox Right Drive";
    public static final String joysticks = "Joysticks";
  }

  public static final class Demo {
    public enum DriveMode {
      OFF, SLOW_ROTATE_ONLY, SLOW_DRIVE
    }

    public static final boolean inDemoMode = false;
    public static final DriveMode driveMode = DriveMode.SLOW_DRIVE;

    public static final double driveScaleFactor = 0.15;
    public static final double rotationScaleFactor = 0.1;
  }

  public static final boolean driveTuningMode = false;
  public static final boolean steeringTuningMode = false;

  public enum DriveDegradedMode {
    normal, sideMotorsOnly, centerMotorsOnly
  }
  public static final DriveDegradedMode driveDegradedMode = DriveDegradedMode.sideMotorsOnly;

  public static final int falconEncoderUnits = 2048;
  public static final double inchesToMeters = 0.0254;
  public static final double feetToMeters = inchesToMeters * 12;
  public static final int fastStatusPeriodBaseMs = 13;
  public static final int shuffleboardStatusPeriodBaseMs = 75;
  public static final int slowStatusPeriodBaseMs = 180;
  public static final int verySlowStatusPeriodSparkBaseMs = 1000;
  public static final int fastStatusPeriodMaxMs = 18;
  public static final int shuffleboardStatusPeriodMaxMs = 90;  // for interactive response
  public static final int slowStatusPeriodMaxMs = 255;
  public static final int controllerConfigTimeoutMs = 50;

  public static final class DriveConstants {
    public static final int frontRightDriveID = 18;
    public static final int frontRightRotationID = 15;
    public static final int frontRightCANID = 0;
    public static final int rearRightDriveID = 19;
    public static final int rearRightRotationID = 22;
    public static final int rearRightCANID = 0;
    public static final int frontLeftDriveID = 17;
    public static final int frontLeftRotationID = 21;
    public static final int frontLeftCANID = 0;
    public static final int rearLeftDriveID = 16;
    public static final int rearLeftRotationID = 20;
    public static final int rearLeftCANID = 0;

    public static final int encoderResolution = 2048;
    
    // full length of drivebase divided by 2 for distance between wheels
    public static final double distWheelMetersX = OrangeMath.inchesToMeters(29.5/2); // 29.5 in
    public static final double distWheelMetersY = OrangeMath.inchesToMeters(29.5/2); // 29.5 in

    // wheel location constants
    public static final Translation2d frontLeftWheelLocation = new Translation2d(distWheelMetersX, distWheelMetersY);
    public static final Translation2d frontRightWheelLocation = new Translation2d(distWheelMetersX, -distWheelMetersY);
    public static final Translation2d backLeftWheelLocation = new Translation2d(-distWheelMetersX, distWheelMetersY);
    public static final Translation2d backRightWheelLocation = new Translation2d(-distWheelMetersX, -distWheelMetersY);

    public static final double disableBreakSec = 2.0;

    // top speed at full motor output is 91 rot/sec with voltage comp at 11.5 volts
    // however, setting the max speed to 91 only allows us to reach 86 due to insufficent kV
    public static final double maxSpeedMetersPerSecond = OrangeMath.falconRotationsToMeters(91,
        OrangeMath.inchesToMeters(OrangeMath.getCircumference(Drive.wheelDiameterInches)),
        Drive.gearRatio);

    public static final double maxRotationSpeedRadSecond = 12.2718;  // physical limit of the bot

    public static final double stoppedVelocityThresholdFtPerSec = 0.5;
    public static final double movingVelocityThresholdFtPerSec = 1.5;

    public static final double drivePolarDeadband = 0.06;
    public static final double twistDeadband = 0.08;

    // Values for auto balance
    public static final double autoBalanceFlatPower = 0.27;
    public static final double autoBalanceRampPower = 0.2;
    public static final double autoBalanceAdjustmentPower = 0.055;
    public static final double chargeStationTiltedMinDeg = 10.0;
    public static final double chargeStationDroppingDeg = 1.5;
    public static final double rampImpulseSec = 0.9;  // time for gyro to stabilize
    public static final double droppingSec = 0.35;
    public static final double levelingSec = 0.3;
    public static final double chargeStationBalancedMaxDeg = 2.0;
    public static final double autoBalanceFlatTimeoutSec = 2.5;
    public static final double autoBalanceTimeoutSec = 15.0;
    public static final double clawTimedOuttake = 1.1;

    public static final double autoDriveOverChargeFlatMaxDeg = 3.0;
    public static final double autoDriveOverChargeFlatSec = 0.5;
    public static final double autoDriveOverChargeTimeoutSec = 6.0;

    public static final double spinoutCenterPower = 1.0;
    public static final double spinoutCornerPower = 0.75;

    public static final double autoChargePower = 0.5;

    public static final double doubleSubstationLoadDistanceInches = 32.5;
    public static final double doubleSubstationMinAprilTagInches = 42;

    public static final class Manual {

      public static final double joystickDriveDeadband = 0.1;
      public static final double joystickRotateLeftDeadband = 0.52;  // don't go below 0.2
      public static final double joystickRotateRightDeadband = 0.35;  // don't go below 0.2

      public static final double xboxDriveDeadband = 0.1;
      public static final double xboxRotateDeadband = 0.2;
      public static final double maxManualRotation = 0.34;
      public static final double inhibitPseudoAutoRotateAngularVelocity = 0.05;
      
      public static final double spinoutRotateDeadBand = 0.9;
      public static final double spinoutMinAngularVelocity = 0.5; // looks like radians per second but we don't know
      public static final double spinoutActivationSec = 0.35;
      public static final double spinoutMinAngularVelocity2 = 0.25;
      public static final double spinout2ActivationSec = 0.2;
    }

    public static final class Auto {

      // Values for autonomous path finding
      public static final double autoMaxSpeedMetersPerSecond = 0.75 * DriveConstants.maxSpeedMetersPerSecond;

      // acceleration off the line is 109 rotations per sec^2
      // acceleration in the mid-range is 46.8 rotations per sec^2
      public static final double autoMaxAccelerationMetersPerSec2 = 0.75 * OrangeMath.falconRotationsToMeters(73,
          OrangeMath.inchesToMeters(OrangeMath.getCircumference(Drive.wheelDiameterInches)),
          Drive.gearRatio);

      public static final double autoRotkP = 0.008;
      public static final double autoRotkD = 0.0004;
      public static final double minAutoRotateStoppedPower = 0.03;
      public static final double minAutoRotateMovingPower = 0.01;
      public static final double rotateStoppedToleranceDegrees = 0.5;
      public static final double rotateMovingToleranceDegrees = 1.5;
      public static final double slowMovingAutoRotate = 0.5;
      public static final double fastMovingAutoRotate = 0.32;
      public static final double fastMovingFtPerSec = 3;
    }

    public static final class Tip {

      public static final double highVelocityFtPerSec = 6.0;
      public static final double lowVelocityFtPerSec = 3.0;
      public static final double highAccFtPerSec2 = 8.0;
      public static final double lowAccFtPerSec2 = 4.0;
      public static final double velAccDiffMaxDeg = 30;
      public static final double highPowerOff = 0.4;
      public static final double lowPowerOff = 0.19;
      public static final double highSpeedSteeringChangeMaxDegrees = 20;
      public static final double velocityHistorySeconds = 0.1;

    }

    public static final class Rotation { 
      // For tuning, graph Duty Cycle Position in the REV Hardware Client
      public static final double kP = 0.03;
      public static final double kD = 0.0;

      public static final double configCLosedLoopRamp = 0.08;
      public static final double maxPower = 0.5; // reduce gear wear and overshoot

      public static final double configVoltageCompSaturation = 11.5;
      public static final boolean enableVoltageCompensation = true;

      public static final int freeLimit = 40;
      public static final int stallLimit = 5; //Change

      public static final double allowableClosedloopError = 0.35;  // degrees
    }

    public static final class Drive {

      // Talon FXs with Phoenix 6 do not currently honor ramp rates!
      public static final double closedLoopRampSec = 0.08;  // used for auto and manual acceleration/deceleration
      public static final double openLoopRampSec = 0.08;  // only used when stopping, including letting go of the drive stick

      public static final double voltageCompSaturation = 12.0;

      public static final double brakeModeDeadband = 0.01;

      public static final double statorLimit = 60;

      public static final int currentLimit = 40;
      public static final int secondaryCurrentLimit = 100;

      // when supply threshold is exceeded for the time, drop the current to the limit
      public static final boolean supplyEnabled = true;
      public static final double supplyLimit = 40;
      public static final double supplyThreshold = 60;
      public static final double supplyTime = 2.0;

      public static final double wheelDiameterInches = 3.9;
      public static final double gearRatio = 7.80; // drive gear ratio
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kIZone = 0.0;
      public static final double kD = 0.0;
      public static final double kF = 11.0; // TODO
      public static final double kS = 0; // TODO

      // Feed Forward parameters for Drive PID
      public static final class FeedForward {
        public static final double[] voltsAtSpeedThresholds;
        static {
          voltsAtSpeedThresholds = new double[4];
          voltsAtSpeedThresholds[0] = 11.0;
          voltsAtSpeedThresholds[1] = 11.0;
          voltsAtSpeedThresholds[2] = 11.0;
          voltsAtSpeedThresholds[3] = 11.0;
        }
        public static final double[] feedForwardRPSThreshold;
        static {
          // define speed at which each voltage value will be used
          feedForwardRPSThreshold = new double[4];
          // values must be in ascending order
          feedForwardRPSThreshold[0] = 0.0; // Must be zero
          feedForwardRPSThreshold[1] = 30.0;
          feedForwardRPSThreshold[2] = 50.0;
          feedForwardRPSThreshold[3] = 80.0;
        }
      }
    }

    public static final class Trajectory {

      public static final class PIDXY {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
      }

      public static final class PIDR {
        public static final double kP = 2.0;
        public static final double kI = 0;
        public static final double kD = 0.01;
      }
    }
  }

  public enum WheelPosition {
    // construction of SwerveDriveKinematics is dependent on this enum

    FRONT_RIGHT(0), FRONT_LEFT(1), BACK_LEFT(2), BACK_RIGHT(3);

    public int wheelNumber;

    WheelPosition(int id) {
      wheelNumber = id;
    }
  }
  
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
