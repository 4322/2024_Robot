// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotChooser.RobotChooser;
import frc.robot.RobotChooser.RobotChooserInterface;
import frc.robot.shooting.FiringSolution;
import frc.utility.CanBusUtil;
import frc.utility.OrangeMath;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum RobotType {
    // Drivebase for testing
    NEMO,

    // 2024 Competition Robot
    CRUSH
  }

  public static final RobotType currentRobot = RobotType.CRUSH;
  public static final Mode currentMode = Mode.REAL;

  // Must be below currentRobot to initialize properly
  private static RobotChooserInterface robotSpecificConstants =
      RobotChooser.getInstance().getConstants();
  public static double noteRadiusInches = 7;

  public static final boolean debug = false;

  public static final boolean driveEnabled = true;
  public static final boolean intakeEnabled = true;
  public static final boolean intakeDeployerEnabled = true;
  public static final boolean gyroEnabled = true;
  public static final boolean tunnelEnabled = true;
  public static final boolean outtakeEnabled = true;
  public static final boolean outtakePivotEnabled = true;
  public static final boolean sensorsEnabled = true;
  public static final boolean ledEnabled = true;
  public static final boolean joysticksEnabled = false;
  public static final boolean xboxEnabled = true;

  public static final boolean intakeLimeLightEnabled = true;
  public static final boolean outtakeLimeLightEnabled = true;

  public static final boolean speakerCentricEnabled = true;
  public static final boolean spinoutCenterEnabled = true; // center rotate burst of power
  public static final boolean spinoutCornerEnabled = true;
  public static final boolean psuedoAutoRotateEnabled = true;
  public static final String driveInputScaling = DriveInputScalingStrings.quadratic;
  public static final String rotateInputScaling = RotateInputScalingStrings.linear;
  public static final double rotateInputPowerScaling = 1.0;
  public static final String controllerType = ControllerTypeStrings.xboxLeftDrive;

  public static final class DriveInputScalingStrings {
    public static final String linear = "Linear";
    public static final String quadratic = "Quadratic";
    public static final String cubic = "Cubic";
  }

  public static final class RotateInputScalingStrings {
    public static final String linear = "Linear";
    public static final String squareRoot = "Square Root";
    public static final String quadratic = "Quadratic";
    public static final String power = "Power";
  }

  public static final class ControllerTypeStrings {
    public static final String none = "None";
    public static final String xboxLeftDrive = "Xbox Left Drive";
    public static final String xboxRightDrive = "Xbox Right Drive";
    public static final String joysticks = "Joysticks";
  }

  public static final class Demo {
    public enum DriveMode {
      OFF,
      SLOW_ROTATE_ONLY,
      SLOW_DRIVE
    }

    public static final boolean inDemoMode = false;
    public static final DriveMode driveMode = DriveMode.SLOW_DRIVE;

    public static final double driveScaleFactor = 0.15;
    public static final double rotationScaleFactor = 0.1;
  }

  public static final boolean driveTuningMode = false;
  public static final boolean steeringTuningMode = false;
  public static final boolean outtakeTuningMode = false;

  public enum DriveDegradedMode {
    normal,
    sideMotorsOnly,
    centerMotorsOnly
  }

  public static final DriveDegradedMode driveDegradedMode = DriveDegradedMode.sideMotorsOnly;

  public static final double inchesToMeters = 0.0254;
  public static final double feetToMeters = inchesToMeters * 12;
  public static final int fastStatusPeriodBaseMs = 13;
  public static final int shuffleboardStatusPeriodBaseMs = 75;
  public static final int slowStatusPeriodBaseMs = 180;
  public static final int verySlowStatusPeriodSparkBaseMs = 1000;
  public static final int fastStatusPeriodMaxMs = 18;
  public static final int shuffleboardStatusPeriodMaxMs = 90; // for interactive response
  public static final int slowStatusPeriodMaxMs = 255;
  public static final int controllerConfigTimeoutMs = 50;
  public static final boolean inShotTuning = false;
  public static final double fieldWidthMeters = 8;

  public static final class DriveConstants {

    // robot radius
    public static final double distWheelMetersR =
        Math.sqrt(
            (robotSpecificConstants.getDistWheelMetersX()
                    * robotSpecificConstants.getDistWheelMetersX())
                + (robotSpecificConstants.getDistWheelMetersY()
                    * robotSpecificConstants.getDistWheelMetersY()));

    // wheel location constants
    public static final Translation2d frontLeftWheelLocation =
        new Translation2d(
            robotSpecificConstants.getDistWheelMetersX(),
            robotSpecificConstants.getDistWheelMetersY());
    public static final Translation2d frontRightWheelLocation =
        new Translation2d(
            robotSpecificConstants.getDistWheelMetersX(),
            -robotSpecificConstants.getDistWheelMetersY());
    public static final Translation2d backLeftWheelLocation =
        new Translation2d(
            -robotSpecificConstants.getDistWheelMetersX(),
            robotSpecificConstants.getDistWheelMetersY());
    public static final Translation2d backRightWheelLocation =
        new Translation2d(
            -robotSpecificConstants.getDistWheelMetersX(),
            -robotSpecificConstants.getDistWheelMetersY());

    public static final double disableBreakSec = 2.0;

    public static final double stoppedVelocityThresholdMetersPerSec = 0.1524;
    public static final double movingVelocityThresholdMetersPerSec = 0.4572;

    public static final double spinoutCenterPower = 1.0;
    public static final double spinoutCornerPower = 0.75;

    public static final class Manual {

      public static final double joystickDriveDeadband = 0.1;
      public static final double joystickRotateLeftDeadband = 0.52; // don't go below 0.2
      public static final double joystickRotateRightDeadband = 0.35; // don't go below 0.2

      public static final double xboxDriveDeadband = 0.17; // was 0.1 with a better controller
      public static final double xboxRotateDeadband = 0.2;
      public static final double maxManualRotation = 0.25;
      public static final double inhibitPseudoAutoRotateAngularVelocity = 0.05;

      public static final double spinoutRotateDeadBand = 0.9;
      public static final double spinoutMinAngularVelocity =
          0.5; // looks like radians per second but we don't know
      public static final double spinoutActivationSec = 0.35;
      public static final double spinoutMinAngularVelocity2 = 0.25;
      public static final double spinout2ActivationSec = 0.2;
    }

    public static final class Auto {

      // Values for autonomous path finding
      public static final double autoMaxModuleSpeedMetersPerSecond =
          robotSpecificConstants.getMaxSpeedMetersPerSec();

      // acceleration off the line is 109 rotations per sec^2
      // acceleration in the mid-range is 46.8 rotations per sec^2
      public static final double autoMaxAccelerationMetersPerSec2 =
          0.5
              * OrangeMath.falconRotationsToMeters(
                  73,
                  OrangeMath.inchesToMeters(OrangeMath.getCircumference(Drive.wheelDiameterInches)),
                  robotSpecificConstants.getDriveGearRatio());

      public static final double minAutoRotateStoppedPower =
          robotSpecificConstants.getMinAutoRotateStoppedPower();
      public static final double minAutoRotateMovingPower =
          robotSpecificConstants.getminAutoRotateMovingPower();
      public static final double rotateStoppedToleranceDegrees = 0.5;
      public static final double rotateMovingToleranceDegrees = 1.5;
      public static final double slowMovingAutoRotate = 0.5;
      public static final double fastMovingAutoRotate = 0.32;
      public static final double fastMovingMetersPerSec = 0.9144;
    }

    public static final class Tip {

      public static final double highVelocityMetersPerSec = 1.8288;
      public static final double lowVelocityMetersPerSec = 0.9144;
      public static final double highAccMetersPerSec2 = 2.4384;
      public static final double lowAccMetersPerSec2 = 1.2192;
      public static final double velAccDiffMaxDeg = 30;
      public static final double highPowerOff = 0.4;
      public static final double lowPowerOff = 0.19;
      public static final double highSpeedSteeringChangeMaxDegrees = 20;
      public static final double velocityHistorySeconds = 0.1;
    }

    public static final class Rotation {

      public static final boolean supplyEnabled = true;
      public static final boolean statorEnabled = true;
      public static final double supplyLimit = 30;
      public static final double statorLimit = 45;

      public static final double configCLosedLoopRamp = 0.08;
      public static final double maxPower = 6; // reduce gear wear and overshoot

      public static final double configVoltageCompSaturation = 11.5;

      public static final int freeLimit = 40;
      public static final int stallLimit = 5; // TODO

      public static final double allowableClosedloopError = 0.35; // degrees
      public static final double[] CANCoderOffsetRotations;

      static {
        CANCoderOffsetRotations = new double[4];
        CANCoderOffsetRotations[WheelPosition.FRONT_RIGHT.wheelNumber] = 0.685791;
        CANCoderOffsetRotations[WheelPosition.FRONT_LEFT.wheelNumber] = 0.535517578125;
        CANCoderOffsetRotations[WheelPosition.BACK_RIGHT.wheelNumber] = 0.705322265625 - 0.25;
        CANCoderOffsetRotations[WheelPosition.BACK_LEFT.wheelNumber] = 0.411376953125 + 0.25;
      }
    }

    public static final class Drive {

      // Talon FXs with Phoenix 6 do not currently honor ramp rates!
      public static final double closedLoopRampSec =
          0.08; // used for auto and manual acceleration/deceleration
      public static final double openLoopRampSec =
          0.08; // only used when stopping, including letting go of the drive
      // stick

      public static final double voltageCompSaturation = 12.0;

      public static final double brakeModeDeadband = 0.01;

      public static final int currentLimit = 40;
      public static final int secondaryCurrentLimit = 80;

      public static final double wheelDiameterInches = 4.0;

      public static final String canivoreName = "Clockwork";
      public static final int pigeonID = 10;

      public static final int frontLeftEncoderID = 11;
      public static final int rearLeftEncoderID = 12;
      public static final int frontRightEncoderID = 13;
      public static final int rearRightEncoderID = 14;

      // when supply threshold is exceeded for the time, drop the current to the limit
      public static final double statorLimit = 60;
      public static final boolean supplyEnabled = true;
      public static final boolean statorEnabled = true;
      public static final double supplyLimit = 40;
      public static final double supplyThreshold = 60;
      public static final double supplyTime = 2.0;
    }

    public static final double autoFeedMoveSpeed = 1;
  }

  public static final class EncoderInitializeConstants {
    public static final double initializedRotationsFlag = 4322.0; // must be a very high number
    public static final double initializedRotationsTolerance = 5.0;
    public static final double absEncoderMaxZeroingThreshold = 0.95;
  }

  public static final class OuttakeConstants {
    public static final int leftOuttakeDeviceID = 5;
    public static final int rightOuttakeDeviceID = 4;
    public static final int pivotDeviceID = 6;
    public static final int pivotEncoderID = 8;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.124;
    public static final double kS = 0.35;

    public static final double openLoopRampSec = 0;
    public static final double closedLoopRampSec = 0;
    public static final int gearRatioMotorToWheel = 0;
    public static final double gearReductionEncoderToMotor = (30.0 / 28.0) * 125.0;
    public static final double voltPerRPS =
        0; // since we likely aren't going to adjust the speed, it's likely safe to
    // not interpolate
    public static final boolean supplyEnabled = true;
    public static final boolean statorEnabled = true;
    public static final double shooterSupplyLimit = 40;
    public static final double shooterStatorLimit = 80;
    public static final double shooterSupplyCurrentThreshold = 50;
    public static final double shooterSupplyTimeThreshold = 1.5;

    public static final double pivotSupplyLimit = 30;
    public static final double pivotStatorLimit = 45;

    public static final double pivotkD = 0;
    public static final double pivotkI = 0;
    public static final double pivotkP = 3.0;
    public static final double pivotkFF = 0;

    public static final double maxVelRotationsPerSec = 85;
    public static final boolean enableFOC = false;
    public static final double pivotClosedLoopSec = 0.1;
    public static final boolean limitForwardMotion = true;
    public static final boolean limitReverseMotion = true;
    public static final double forwardSoftLimitThresholdRotations = 120;
    public static final double reverseSoftLimitThresholdRotations = 10;
    public static final double pivotPeakForwardVoltage = 10;
    public static final double pivotPeakReverseVoltage = -10;

    public static final double defaultPivotPositionRotations = 0;
    public static final double ejectOuttakeRPS = 0.0; // TODO
    public static final double subwooferShotMag = 0.0;
    public static final double subwooferShotDeg = 0.0;
    public static final double subwooferPivotPositionRotations = 0.0;
    public static final double subwooferOuttakeRPS = 0.0;

    public static final double topOuttakeRPS = 0;
    public static final double bottomOuttakeRPS = 0;
    public static final double outtakeToleranceRPS = 0;
    public static final double pivotToleranceRotations = 0;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorID = 7;
    public static final int deployMotorID = 2;
    public static final int deployEncoderID = 9;

    public static final class IntakeConfig {
      public static final double updateHz =
          OrangeMath.msAndHzConverter(CanBusUtil.nextSlowStatusPeriodMs());
      public static final double timeoutMs = 50;
      public static final double intakeFeedVoltage = 11.0;
      public static final double intakeEjectVoltage = -5.0;
      public static final boolean supplyEnabled = true;
      public static final boolean statorEnabled = true;
      public static final double supplyLimit = 30;
      public static final double statorLimit = 45;
    }

    public static final class DeployConfig {
      public static final double kP = 8.0;
      public static final double slowPos = 0.2;
      public static final double openLoopRamp = 0;
      public static final double updateHz =
          OrangeMath.msAndHzConverter(CanBusUtil.nextSlowStatusPeriodMs());
      public static final double timeoutMs = 50;

      public static final double peakForwardVoltage = 1.5; // anything higher will slip the belt
      public static final double peakReverseVoltage = -2.5;

      public static final boolean supplyEnabled = true;
      public static final boolean statorEnabled = true;
      public static final double supplyLimit = 30;
      public static final double statorLimit = 45;

      public static final double deployTargetPosition = 0.0;
      public static final double retractTargetPosition = 0.6;
      public static final double atTargetTolerance = 0.03;
      public static final double deployFallTolerance = 0.08;
      public static final double maxRotationsPerSec = 0.2;
    }

    public static final class Logging {
      public static final String key = "Intake/";
      public static final String feederKey = "Intake/Feeder/";
      public static final String feederHardwareOutputsKey = "Intake/Feeder/Hardware/";
      public static final String deployerKey = "Intake/Deployer/";
      public static final String deployerHardwareOutputsKey = "Intake/Deployer/Hardware/";
    }
  }

  public static final class BeamBreakConstants {
    public static final int tunnelBeamBreakID = 6;
    public static final int intakeBeamBreakID = 5;

    public static final class Logging {
      public static final String key = "Sensors/";
      public static final String hardwareOutputsKey = "Sensors/Hardware/";
    }
  }

  public static final class TunnelConstants {
    public static final int tunnelMotorID = 3;
    public static final boolean supplyEnabled = true;
    public static final boolean statorEnabled = true;
    public static final double supplyLimit = 30;
    public static final double statorLimit = 45;

    public static final double desiredVoltage = 4.0;
    public static final double peakVoltage = 6.0;

    public static final class Logging {
      public static final String key = "Tunnel/";
      public static final String hardwareOutputsKey = "Tunnel/Hardware/";
    }
  }

  public static final class LimelightConstants {
    public static final double visionOdometryTolerance = 0.5;
    public static final double reverseOdometryOverrideTolerance = 1.0;
    public static final int numTargetsToUseReverseOdom = 2;
    // TODO: Values need to be updated to the limelight itself.
    public static final double outtakeLimelightAngle = 25;
    public static final double outtakeLimelightHeight = OrangeMath.inchesToMeters(26.231);
    public static final double outtakeLimelightXOffsetMeters = 0.0;
    public static final double outtakeLimelightYOffsetMeters = OrangeMath.inchesToMeters(-3.387);
    public static final String outtakeLimelightName = "limelight-shooter";

    public static final double intakeLimelightAngle = -25;
    public static final double intakeLimelightHeight = OrangeMath.inchesToMeters(25.237216);
    public static final double intakeLimeLightXOffsetMeters = 0.0;
    public static final double intakeLimelightYOffsetMeters = OrangeMath.inchesToMeters(7.835154);
    public static final String intakeLimelightName = "limelight-note";

    // Target alignment values
    public static final double substationMinLargeTargetArea =
        1.8; // small target is < 1.2 against substation
    public static final double substationOffsetDeg =
        -10.02; // account for limelight being to the left of actual robot
    // center
    public static final double substationTargetToleranceDeg =
        17.5; // human player can drop game piece to the side

    // List of tape pipelines (should only be 1 for now)

    // Map of pipelines and tag heights
    // Map of pipelines andcenter of tag heights
    public static final double sourceZoneAprilTagHeight = 53.38;
    public static final double speakerAprilTagHeight = 57.13;
    public static final double ampZoneAprilTagHeight = 53.38;
    public static final double stageAprilTagHeight = 52.00;
    public static final Map<Integer, Double> tagPipelinesHeights =
        Map.ofEntries(
            Map.entry(1, sourceZoneAprilTagHeight),
            Map.entry(2, sourceZoneAprilTagHeight),
            Map.entry(3, speakerAprilTagHeight),
            Map.entry(4, speakerAprilTagHeight),
            Map.entry(5, ampZoneAprilTagHeight),
            Map.entry(6, ampZoneAprilTagHeight),
            Map.entry(7, speakerAprilTagHeight),
            Map.entry(8, speakerAprilTagHeight),
            Map.entry(9, sourceZoneAprilTagHeight),
            Map.entry(10, sourceZoneAprilTagHeight),
            Map.entry(11, stageAprilTagHeight),
            Map.entry(12, stageAprilTagHeight),
            Map.entry(13, stageAprilTagHeight),
            Map.entry(14, stageAprilTagHeight),
            Map.entry(15, stageAprilTagHeight),
            Map.entry(16, stageAprilTagHeight));
  }

  public static final class LED {
    public static final int totalLEDs = 0; // TODO: find umber of INDIVIDUAL LEDs mounted on robot
    public static final int CANdleID = 3;
  }

  public static final class FieldConstants {
    public static double xSpeakerPosM;
    public static double ySpeakerPosM;

    static {
      if (DriverStation.getAlliance()
          .get()
          .equals(Alliance.Blue)) { // Account for origin remaining same between blue and red
        xSpeakerPosM = 0;
        ySpeakerPosM = 5.546;
      } else {
        xSpeakerPosM = 16.591;
        ySpeakerPosM = 5.546;
      }
    }

    public static final double xCenterLineM = 8.2955;
  }

  public static final class FiringSolutions {
    // TODO: update speeds and angles
    // shot mag/deg don't matter since these are used for setting speed/angle only
    public static FiringSolution SubwooferBase = new FiringSolution(0, 0, 0, 0);
    public static FiringSolution N6 = new FiringSolution(0, 0, 0, 0);
    public static FiringSolution N7 = new FiringSolution(0, 0, 0, 0);
    public static FiringSolution N8 = new FiringSolution(0, 0, 0, 0);
    public static FiringSolution TS = new FiringSolution(0, 0, 0, 0);
    public static FiringSolution MS = new FiringSolution(0, 0, 0, 0);
    public static FiringSolution BS = new FiringSolution(0, 0, 0, 0);
  }

  public enum WheelPosition {
    // construction of SwerveDriveKinematics is dependent on this enum

    FRONT_RIGHT(0),
    FRONT_LEFT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    public int wheelNumber;

    WheelPosition(int id) {
      wheelNumber = id;
    }
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
