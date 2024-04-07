package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  NetworkTable table;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry ledMode;
  NetworkTableEntry camMode;
  NetworkTableEntry pipeline;

  // SHUFFLEBOARD
  ShuffleboardTab tab;
  GenericEntry distanceToTargetX;
  GenericEntry distanceToTargetY;
  GenericEntry targetVisible;

  String name;
  boolean backward;
  boolean isTestSubsystem;
  double limeHeight;
  double limeAngle;
  boolean enabled;
  int currentPipeline = -1;
  boolean isNetworkTableConnected;
  Map<Double, LimelightHelpers.LimelightTarget_Fiducial> llFiducialMap =
      new HashMap<Double, LimelightHelpers.LimelightTarget_Fiducial>();
  Pose2d limelightPose = new Pose2d();

  // the distance from where you want to calculate from
  // should always be calculated with WPI coordinates (front is positive X)
  Translation2d offset;

  private static Limelight outtakeLimelight;
  private static Limelight intakeLimelight;

  public static Limelight getIntakeInstance() {
    if (intakeLimelight == null) {
      // Measuring from front of bumpers
      // Limelight name must match limelight tool
      intakeLimelight =
          new Limelight(
              LimelightConstants.intakeLimelightName,
              Constants.LimelightConstants.intakeLimelightHeight,
              Constants.LimelightConstants.intakeLimelightAngle,
              Constants.LimelightConstants.intakeLimeLightXOffsetMeters,
              Constants.LimelightConstants.intakeLimelightYOffsetMeters,
              false,
              false,
              Constants.intakeLimeLightEnabled);
    }
    return intakeLimelight;
  }

  public static Limelight getOuttakeInstance() {
    if (outtakeLimelight == null) {
      // Measuring from back of bumpers
      // Limelight name must match limelight tool
      outtakeLimelight =
          new Limelight(
              LimelightConstants.outtakeLimelightName,
              Constants.LimelightConstants.outtakeLimelightHeight,
              Constants.LimelightConstants.outtakeLimelightAngle,
              Constants.LimelightConstants.outtakeLimelightXOffsetMeters,
              Constants.LimelightConstants.outtakeLimelightYOffsetMeters,
              true,
              false,
              Constants.outtakeLimeLightEnabled);
    }
    return outtakeLimelight;
  }

  private Limelight(
      String limelightName,
      double limelightHeightMeters,
      double limelightAngleDegrees,
      double xOffsetMeters,
      double yOffsetMeters,
      boolean facingBackward,
      boolean isTestSubsystem,
      boolean enabled) {
    name = limelightName;
    limeHeight = limelightHeightMeters;
    limeAngle = limelightAngleDegrees;
    offset = new Translation2d(xOffsetMeters, yOffsetMeters);
    backward = facingBackward;
    this.isTestSubsystem = isTestSubsystem;
    this.enabled = enabled;

    if (enabled) {
      table = NetworkTableInstance.getDefault().getTable(name);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      ledMode = table.getEntry("ledMode");
      camMode = table.getEntry("camMode");
      pipeline = table.getEntry("pipeline");

      // If speaker centric is disabled, assume we aren't using odometry in match
      // Switch outtake limeligt to normal vision pipeline so driver can have better vision of
      // speaker
      if (limelightName.equals(Constants.LimelightConstants.outtakeLimelightName)) {
        // TODO: look at LL pipeline numbers for april tags and normal vision
        if (!Constants.speakerCentricEnabled) {
          switchPipeline(1); // switch to normal vision pipeline
        } else {
          switchPipeline(0); // switch to April Tag pipeline
        }
      }

      if (Constants.debug) {
        tab = Shuffleboard.getTab(name);
        targetVisible =
            tab.add("Target Visible", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 0)
                .getEntry();
        distanceToTargetX = tab.add("Target X", 0).withPosition(0, 1).getEntry();
        distanceToTargetY = tab.add("Target Y", 0).withPosition(0, 2).getEntry();
      }
    }
  }

  @Override
  public void periodic() {
    if (enabled) {
      // Only refresh odometry for outtake limelight to know which fiducials are visible
      if (name == Constants.LimelightConstants.outtakeLimelightName) {
        refreshOdometry();
        updateBotposeWpiBlue();
      }
      
      if (Constants.debug) {
        boolean visible = getTargetVisible();
        targetVisible.setBoolean(visible);
        if (visible) {
          Translation2d targetPos = getTargetPosRobotRelative();
          distanceToTargetX.setDouble(targetPos.getX());
          distanceToTargetY.setDouble(targetPos.getY());
        }
      }

      if (table == null) {
        isNetworkTableConnected = false;
      } else {
        isNetworkTableConnected = true;
      }
    }
  }

  public void updateBotposeWpiBlue() {
    if (enabled && isNetworkTableConnected) {
      if (getTargetVisible()) {
        limelightPose = LimelightHelpers.getBotPose2d_wpiBlue(name);
      }
    }
  }

  public Pose2d getBotposeWpiBlue() {
    return limelightPose;
  }

  public int getNumTargets() {
    if (enabled && isNetworkTableConnected) {
      final int numTargets = llFiducialMap.size();
      Logger.recordOutput(name + "/NumTargets", numTargets);
      return numTargets;
    }
    return 0;
  }

  public double getTotalLatency() {
    if (enabled && isNetworkTableConnected) {
      return LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name);
    }
    return 0;
  }

  public void refreshOdometry() {
    String json = LimelightHelpers.getJSONDump(name);
    llFiducialMap.clear();
    int nextPos = 0;
    while ((nextPos = json.indexOf("\"fID\":", nextPos)) != -1) {
      int startIndex = nextPos + 6;
      nextPos++; // don't get stuck in a loop if there is no tx
      if ((nextPos = json.indexOf(',', nextPos)) != -1) {
        LimelightHelpers.LimelightTarget_Fiducial fiducial =
            new LimelightHelpers.LimelightTarget_Fiducial();
        fiducial.fiducialID = Double.valueOf(json.substring(startIndex, nextPos));
        if ((nextPos = json.indexOf("\"tx\":", nextPos)) != -1) {
          startIndex = nextPos + 5;
          if ((nextPos = json.indexOf(',', nextPos)) != -1) {
            fiducial.tx = Double.parseDouble(json.substring(startIndex, nextPos));
            if ((nextPos = json.indexOf("\"ty\":", nextPos)) != -1) {
              startIndex = nextPos + 5;
              if ((nextPos = json.indexOf(',', nextPos)) != -1) {
                fiducial.ty = Double.parseDouble(json.substring(startIndex, nextPos));
                llFiducialMap.put(fiducial.fiducialID, fiducial);
              }
            }
          }
        }
      }
    }
  }

  public LimelightHelpers.LimelightTarget_Fiducial getTag(double fID) {
    return llFiducialMap.get(fID);
  }

  public boolean getSpecifiedTagVisible(int fID1) {
    return getTag(fID1) != null;
  }

  public double getHorizontalDegToTarget() {
    if (enabled && isNetworkTableConnected) {
      return tx.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getVerticalDegToTarget() {
    if (enabled && isNetworkTableConnected) {
      return ty.getDouble(0);
    } else {
      return 0;
    }
  }

  public double getTargetArea() {
    if (enabled && isNetworkTableConnected) {
      return ta.getDouble(0);
    } else {
      return 0;
    }
  }

  public boolean getTargetVisible() {
    if (enabled && isNetworkTableConnected) {
      return tv.getDouble(0.0) == 1.0;
    } else {
      return false;
    }
  }

  public void setCamMode(CamMode mode) {
    if (enabled && isNetworkTableConnected) {
      if (mode == CamMode.VisionProcessor) {
        camMode.setNumber(0);
      } else if (mode == CamMode.DriverCamera) {
        camMode.setNumber(1);
      }
    }
  }

  public enum CamMode {
    VisionProcessor,
    DriverCamera;
  }

  // All distances use WPILib coordinates (where x is perpendicular to the target and y
  // is parallel to the target)
  public double getTargetHeight(int pipelineIdx, double yDeg) {
    if (LimelightConstants.tagPipelinesHeights.containsKey(pipelineIdx)) {
      return LimelightConstants.tagPipelinesHeights.get(pipelineIdx);
    }
    return -1; // invalid pipeline
  }

  // Angles in degrees
  // distance y formula referenced from:
  // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
  public Translation2d calcTargetPos(double targetHeight, double yDeg, double xDeg) {
    double distanceX = (targetHeight - limeHeight) / Math.tan(Math.toRadians(yDeg + limeAngle));
    double distanceY = distanceX * Math.tan(Math.toRadians(xDeg));
    Translation2d toReturn = new Translation2d(distanceX, distanceY).plus(offset);
    if (backward) {
      toReturn = toReturn.rotateBy(Rotation2d.fromDegrees(180));
    }
    return toReturn;
  }

  public Translation2d getTargetPosRobotRelative() {
    if (enabled && isNetworkTableConnected) {
      if (getTargetVisible()) {
        double yDeg = getVerticalDegToTarget();
        double xDeg = getHorizontalDegToTarget();
        int pipelineIdx = (int) pipeline.getInteger(0);
        double targetHeight = getTargetHeight(pipelineIdx, yDeg);
        if (targetHeight != -1) {
          return calcTargetPos(targetHeight, yDeg, xDeg);
        }
        // DataLogManager.log(name + ": Tried to get target pos, but pipline was invalid");
        return new Translation2d(0, 0);
      }
      // DataLogManager.log(name + ": Tried to get target pos, but no target found");
      return new Translation2d(0, 0);
    }
    // DataLogManager.log(name + ": Tried to get target pos, but limelight is disabled");
    return new Translation2d(0, 0);
  }

  public double getHorizontalDistToTarget() {
    double distanceX =
        (getTargetHeight(currentPipeline, getHorizontalDegToTarget()) - limeHeight)
            / Math.tan(Math.toRadians(getHorizontalDegToTarget() + limeAngle));
    double distanceY = distanceX * Math.tan(Math.toRadians(getVerticalDegToTarget()));

    return distanceY;
  }

  public void activateRetroReflective() {
    switchPipeline(0);
  }

  public void activateAprilTag() {
    switchPipeline(1);
  }

  private void switchPipeline(int pipelineIdx) {
    if (enabled && (currentPipeline != pipelineIdx) && isNetworkTableConnected) {
      pipeline.setNumber(pipelineIdx);
      currentPipeline = pipelineIdx;
    }
  }
}
