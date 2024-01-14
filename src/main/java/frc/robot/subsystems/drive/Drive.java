package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheelPosition;
import frc.utility.OrangeMath;
import frc.utility.SnapshotTranslation2D;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.ControllerTypeStrings;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private GyroIO gyro;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private DriveShuffleBoardIO driveShuffleBoard;
  private DriveShuffleBoardIOInputsAutoLogged driveShuffleBoardInputs = new DriveShuffleBoardIOInputsAutoLogged();

  private PIDController rotPID;

  private Timer runTime = new Timer();

  private double latestVelocity;
  private double latestAcceleration;
  private double pitchOffset;

  private ArrayList<SnapshotTranslation2D> velocityHistory = new ArrayList<SnapshotTranslation2D>();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      DriveConstants.frontRightWheelLocation, DriveConstants.frontLeftWheelLocation,
      DriveConstants.backLeftWheelLocation, DriveConstants.backRightWheelLocation);

  private final SwerveDrivePoseEstimator poseEstimator;

  private ShuffleboardTab tab;

  private GenericEntry rotErrorTab;
  private GenericEntry rotSpeedTab;
  private GenericEntry rotkP;
  private GenericEntry rotkD;
  private GenericEntry yawTab;
  private GenericEntry rollTab;
  private GenericEntry pitchTab;
  private GenericEntry botVelocityMag;
  private GenericEntry botAccelerationMag;
  private GenericEntry botVelocityAngle;
  private GenericEntry botAccelerationAngle;
  private GenericEntry driveXTab;
  private GenericEntry driveYTab;
  private GenericEntry rotateTab;
  private GenericEntry odometryX;
  private GenericEntry odometryY;
  private GenericEntry odometryDegrees;
  private GenericEntry angularVel;

  private double lastClosedRampRate = DriveConstants.Drive.closedLoopRampSec;
  private double lastOpenRampRate = DriveConstants.Drive.openLoopRampSec;

  private static Drive driveSubsystem = null;
  public static Drive getInstance() {
    if (driveSubsystem == null) {
      driveSubsystem = new Drive();
    }
    return driveSubsystem;
  }

  private Drive() {
    runTime.start();
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        if (Constants.driveEnabled) {
          swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] =
              new SwerveModule(WheelPosition.FRONT_RIGHT, new SwerveModuleIOMotorControl(WheelPosition.FRONT_RIGHT));
          swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] =
              new SwerveModule(WheelPosition.FRONT_LEFT, new SwerveModuleIOMotorControl(WheelPosition.FRONT_LEFT));
          swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] =
              new SwerveModule(WheelPosition.BACK_RIGHT, new SwerveModuleIOMotorControl(WheelPosition.BACK_RIGHT));
          swerveModules[WheelPosition.BACK_LEFT.wheelNumber] =
              new SwerveModule(WheelPosition.BACK_LEFT, new SwerveModuleIOMotorControl(WheelPosition.BACK_LEFT));
        }
        if (Constants.gyroEnabled) {
          gyro = new GyroIONavX();
        }
        driveShuffleBoard = new DriveShuffleBoardIODataEntry();
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        break;

      // Replayed robot, disable hardware IO implementations
      case REPLAY:
        break;
    }

    // instantiates fake hardware when in replay mode
    if (swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] == null) {
      swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] =
          new SwerveModule(WheelPosition.FRONT_RIGHT, new SwerveModuleIO() {});
      swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] =
          new SwerveModule(WheelPosition.FRONT_LEFT, new SwerveModuleIO() {});
      swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] =
          new SwerveModule(WheelPosition.BACK_RIGHT, new SwerveModuleIO() {});
      swerveModules[WheelPosition.BACK_LEFT.wheelNumber] =
          new SwerveModule(WheelPosition.BACK_LEFT, new SwerveModuleIO() {});
    }

    if (gyro == null) {
      gyro = new GyroIO() {};
    }
    if (driveShuffleBoard == null) {
      driveShuffleBoard = new DriveShuffleBoardIO() {};
    }

    if (Constants.driveEnabled) {
      rotPID = new PIDController(DriveConstants.Auto.autoRotkP, 0, DriveConstants.Auto.autoRotkD);

      if (Constants.gyroEnabled) {  
        // wait for first gyro reading to be received
        try {
          Thread.sleep(2000);
        } catch (InterruptedException e) {
        }  
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), getModulePostitions(), new Pose2d());
        resetFieldCentric(0);
      }

    if (Constants.debug) {
      tab = Shuffleboard.getTab("Drivebase");

      rotErrorTab = tab.add("Rot Error", 0).withPosition(0, 0).withSize(1, 1).getEntry();

      rotSpeedTab = tab.add("Rotation Speed", 0).withPosition(0, 1).withSize(1, 1).getEntry();

      rotkP = tab.add("Rotation kP", DriveConstants.Auto.autoRotkP).withPosition(1, 0)
          .withSize(1, 1).getEntry();

      rotkD = tab.add("Rotation kD", DriveConstants.Auto.autoRotkD).withPosition(2, 0)
          .withSize(1, 1).getEntry();

      yawTab = tab.add("Yaw", 0).withPosition(0, 3).withSize(1, 1).getEntry();

      rollTab = tab.add("Roll", 0).withPosition(1, 1).withSize(1, 1).getEntry();

      pitchTab = tab.add("Pitch", 0).withPosition(2, 1).withSize(1, 1).getEntry();

      botVelocityMag = tab.add("Bot Vel Mag", 0).withPosition(3, 0).withSize(1, 1).getEntry();

      botAccelerationMag = tab.add("Bot Acc Mag", 0).withPosition(3, 1).withSize(1, 1).getEntry();

      botVelocityAngle = tab.add("Bot Vel Angle", 0).withPosition(4, 0).withSize(1, 1).getEntry();

      botAccelerationAngle =
          tab.add("Bot Acc Angle", 0).withPosition(4, 1).withSize(1, 1).getEntry();

      angularVel = tab.add("Angular Vel", 0).withPosition(5, 0).withSize(1, 1).getEntry();

      driveXTab = tab.add("Drive X", 0).withPosition(0, 2).withSize(1, 1).getEntry();

      driveYTab = tab.add("Drive Y", 0).withPosition(1, 2).withSize(1, 1).getEntry();

      rotateTab = tab.add("Rotate", 0).withPosition(1, 3).withSize(1, 1).getEntry();

      odometryX = tab.add("Odometry X", 0).withPosition(3, 2).withSize(1, 1).getEntry();

      odometryY = tab.add("Odometry Y", 0).withPosition(4, 2).withSize(1, 1).getEntry();

      odometryDegrees =
          tab.add("Odometry Degrees", 0).withPosition(2, 2).withSize(1, 1).getEntry();

      }
    }
  }

  // get the yaw angle
  public double getAngle() {
    if (gyro != null && gyroInputs.connected && !gyroInputs.calibrating && Constants.gyroEnabled) {
      return OrangeMath.boundDegrees(gyroInputs.yawAngleDeg);
    } else {
      return 0;
    }
  }

  // Get pitch in degrees. Positive angle is the front of the robot raised.
  public double getPitch() {
    if (gyro != null && gyroInputs.connected && !gyroInputs.calibrating && Constants.gyroEnabled) {
      return gyroInputs.pitchPositionDeg - pitchOffset;
    } else {
      return 0;
    }
  }

  // get the change of robot heading in degrees per sec
  public double getAngularVelocity() {
    if (gyro != null && gyroInputs.connected && !gyroInputs.calibrating && Constants.gyroEnabled) {
      return gyroInputs.yawVelocityDegPerSec;
    } else {
      return 0;
    }
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyroInputs.yawAngleDeg);
}

  @Override
  public void periodic() {
    if (Constants.driveEnabled) {
      // update logs
      driveShuffleBoard.updateInputs(driveShuffleBoardInputs);
      Logger.getInstance().processInputs("DriveShuffleBoard/DriveShuffleBoardInputs", driveShuffleBoardInputs);
      for (SwerveModule module : swerveModules) {
        module.periodic();
      }
      if (Constants.gyroEnabled) {
        gyro.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
      }
      updateVelAcc();

      if (Constants.gyroEnabled) {
        updateOdometry();
      }
      
      if (Constants.debug) {
        if (Constants.gyroEnabled) {
          yawTab.setDouble(getAngle());
          rollTab.setDouble(gyroInputs.rollPositionDeg);
          pitchTab.setDouble(getPitch());
          odometryX.setDouble(getPose2d().getX());
          odometryY.setDouble(getPose2d().getY());
          odometryDegrees.setDouble(getPose2d().getRotation().getDegrees());
          angularVel.setDouble(getAngularVelocity());
        }
        double newRampRate = driveShuffleBoardInputs.accelerationRampRate;
        if (lastClosedRampRate != newRampRate) {
          lastClosedRampRate = newRampRate;
          for (SwerveModule module : swerveModules) {
            module.setClosedRampRate(newRampRate);
          }
        }
        newRampRate = driveShuffleBoardInputs.stoppedRampRate;
        if (lastOpenRampRate != newRampRate) {
          lastOpenRampRate = newRampRate;
          for (SwerveModule module : swerveModules) {
            module.setOpenRampRate(newRampRate);
          }
        }
        for (SwerveModule module: swerveModules) {
          module.updateFFVelocityThreshold(driveShuffleBoardInputs.feedForwardRPSThresholds);
          module.updateFeedForward(driveShuffleBoardInputs.voltsAtSpeedThresholds);
          module.updateVoltsToOvercomeFriction(driveShuffleBoardInputs.voltsToOvercomeFriction);
        }
      }
    }
  }


  // rotation isn't considered to be movement
  public boolean isRobotMoving() {
    if (Constants.driveEnabled) {
      return latestVelocity >= DriveConstants.stoppedVelocityThresholdFtPerSec;
    } else {
      return false;
    }
  }

  public boolean isRobotMovingFast() {
    if (Constants.driveEnabled) {
      return latestVelocity >= DriveConstants.movingVelocityThresholdFtPerSec;
    } else {
      return false;
    }
  }

  public void resetFieldCentric(double offset) {
    if (Constants.driveEnabled && Constants.gyroEnabled && gyro != null) {
      gyro.setAngleAdjustment(gyroInputs.angleAdjustment + gyroInputs.yawAngleDeg + offset);
      pitchOffset = gyroInputs.pitchPositionDeg;
    }
  }

  // this drive function is for regular driving when pivot point is at robot center point
  public void drive(double driveX, double driveY, double rotate) {
    drive(driveX, driveY, rotate, new Translation2d());
  }

  // main drive function accounts for spinout when center point is on swerve modules
  public void drive(double driveX, double driveY, double rotate, Translation2d centerOfRotation) {
    if (Constants.driveEnabled && Constants.gyroEnabled) {

      if (Constants.debug) {
        driveXTab.setDouble(driveX);
        driveYTab.setDouble(driveY);
        rotateTab.setDouble(rotate);
      }
      // convert to proper units
      rotate = rotate * DriveConstants.maxRotationSpeedRadSecond;
      driveX = driveX * DriveConstants.maxSpeedMetersPerSecond;
      driveY = driveY * DriveConstants.maxSpeedMetersPerSecond;

      // ready to drive!
      if ((driveX == 0) && (driveY == 0) && (rotate == 0)) {
        stop();
      } else {
        Rotation2d robotAngle;
          robotAngle = getRotation2d();

        // create SwerveModuleStates inversely from the kinematics
        var swerveModuleStates = kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, rotate, robotAngle), centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
            Constants.DriveConstants.maxSpeedMetersPerSecond);
        for (int i = 0; i < swerveModules.length; i++) {
          swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
      }
    }
  }


  // Rotate the robot to a specific heading while driving.
  // Must be invoked periodically to reach the desired heading.
  public void driveAutoRotate(double driveX, double driveY, double targetDeg) {
    if (Constants.driveEnabled) {

      if (Constants.debug) {
        rotPID.setP(rotkP.getDouble(DriveConstants.Auto.autoRotkP));
        rotPID.setD(rotkD.getDouble(DriveConstants.Auto.autoRotkD));
      }

      // Don't use absolute heading for PID controller to avoid discontinuity at +/- 180 degrees
      double headingChangeDeg = OrangeMath.boundDegrees(targetDeg - getAngle());
      double rotPIDSpeed = rotPID.calculate(0, headingChangeDeg);
      double adjMaxAutoRotatePower;
      double adjMinAutoRotatePower;
      double toleranceDeg;

      // reduce rotation power when driving fast to not lose forward momentum
      if (latestVelocity >= driveShuffleBoardInputs.fastMovingFtPerSec) {
        adjMaxAutoRotatePower = driveShuffleBoardInputs.fastMovingAutoRotatePower;
      } else {
        adjMaxAutoRotatePower = driveShuffleBoardInputs.slowMovingAutoRotatePower;
      }
      // no need to maintain exact heading when driving to reduce wobble
      if (isRobotMoving()) {
        adjMinAutoRotatePower = DriveConstants.Auto.minAutoRotateMovingPower;
        toleranceDeg = Constants.DriveConstants.Auto.rotateMovingToleranceDegrees;
      } else {
        // greater percision when lining up for something
        adjMinAutoRotatePower = DriveConstants.Auto.minAutoRotateStoppedPower;
        toleranceDeg = Constants.DriveConstants.Auto.rotateStoppedToleranceDegrees;
      }

      if (Math.abs(headingChangeDeg) <= toleranceDeg) {
        rotPIDSpeed = 0;  // don't wiggle
      } else if (Math.abs(rotPIDSpeed) < adjMinAutoRotatePower) {
        rotPIDSpeed = Math.copySign(adjMinAutoRotatePower, rotPIDSpeed);
      } else if (rotPIDSpeed > adjMaxAutoRotatePower) {
        rotPIDSpeed = adjMaxAutoRotatePower;
      } else if (rotPIDSpeed < -adjMaxAutoRotatePower) {
        rotPIDSpeed = -adjMaxAutoRotatePower;
      }

      drive(driveX, driveY, rotPIDSpeed);

      if (Constants.debug) {
        rotErrorTab.setDouble(headingChangeDeg);
        rotSpeedTab.setDouble(rotPIDSpeed);
      }
    }
  }

  public void resetRotatePID() {
    if (Constants.driveEnabled) {
      rotPID.reset();
    }
  }

  public void updateOdometry() {
    if (Constants.gyroEnabled) {
      poseEstimator.update(getRotation2d(), getModulePostitions());
    }
  }

  public void updateVision(Pose2d pose, double timestampSeconds) {
    if (Constants.gyroEnabled) {
      poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }
  }

  public void resetOdometry(Pose2d pose) {
    if (Constants.gyroEnabled) {
      poseEstimator.resetPosition(getRotation2d(), getModulePostitions(), pose);
    }
  }

  public void setCoastMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setCoastmode();
      }
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setBrakeMode();
      }
    }
  }

  public void stop() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
    }
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition(); 
  }

  public void setModuleStates(SwerveModuleState[] states) {
    if (Constants.driveEnabled) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeedMetersPerSecond);
      int i = 0;
      for (SwerveModuleState s : states) {
        swerveModules[i].setDesiredState(s);
        i++;
      }
    }
  }

  public SwerveModulePosition[] getModulePostitions() {
    if (Constants.driveEnabled) {
      // wheel locations must be in the same order as the WheelPosition enum values
      return new SwerveModulePosition[] {
          swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.FRONT_LEFT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.BACK_LEFT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.BACK_RIGHT.wheelNumber].getPosition()};
    } else {
      return null;
    }
  }

  public SwerveDriveKinematics getKinematics() {
    if (Constants.driveEnabled) {
      return kinematics;
    } else {
      return null;
    }
  }

  public boolean isPseudoAutoRotateEnabled() {
    if (Constants.driveEnabled) {
      if (Constants.debug) {
        return driveShuffleBoardInputs.psuedoAutoRotateEnabled;
      }
    }
    return Constants.psuedoAutoRotateEnabled;
  }

  public double getMaxManualRotationEntry() {
    if (Constants.driveEnabled) {
      if (Constants.debug) {
        return driveShuffleBoardInputs.maxManualRotatePower;
      }
    }
    return Constants.DriveConstants.Manual.maxManualRotation;
  }

  public String getInputScaling() {
    if (Constants.driveEnabled) {
      if (Constants.debug) {
        return driveShuffleBoardInputs.inputScaling;
      }
    }
    return Constants.driveInputScaling;
  }

  public String getControlType() {
    String controller = Constants.controllerType;
    if (Constants.driveEnabled) {
      if (Constants.debug) {
        controller = driveShuffleBoardInputs.driveControllerType;
      }
    }

    //ignore disabled controllers
    switch (controller) {
      case ControllerTypeStrings.none:
        break;
      case ControllerTypeStrings.joysticks:
        if (!Constants.joysticksEnabled) {
          controller = ControllerTypeStrings.none;
        }
        break;
      case ControllerTypeStrings.xboxLeftDrive:
      case ControllerTypeStrings.xboxRightDrive:
        if (!Constants.xboxEnabled) {
          controller = ControllerTypeStrings.none;
        }
        break;
    }
    return controller;
  }

  private void updateVelAcc() {
    double clock = runTime.get(); // cache value to reduce CPU usage
    double[] currentAngle = new double[4];
    for (int i = 0; i < swerveModules.length; i++) {
      currentAngle[i] = swerveModules[i].getInternalRotationDegrees();
    }

    Translation2d velocityXY = new Translation2d();
    Translation2d accelerationXY = new Translation2d();
    // sum wheel velocity and acceleration vectors
    for (int i = 0; i < swerveModules.length; i++) {
      double wheelAngleDegrees = currentAngle[i];
      velocityXY = velocityXY.plus(new Translation2d(swerveModules[i].getVelocityFeetPerSec(),
          Rotation2d.fromDegrees(wheelAngleDegrees)));
      accelerationXY = accelerationXY.plus(new Translation2d(swerveModules[i].snapshotAcceleration(),
          Rotation2d.fromDegrees(wheelAngleDegrees)));
    }
    latestVelocity = velocityXY.getNorm() / 4;
    latestAcceleration = accelerationXY.getNorm() / 4;

    Logger.getInstance().recordOutput("Drive/BotVelFtPerSec", latestVelocity);
    Logger.getInstance().recordOutput("Drive/BotVelDegrees", velocityXY.getAngle().getDegrees());
    Logger.getInstance().recordOutput("Drive/BotAccFtPerSec2", latestAcceleration);
    Logger.getInstance().recordOutput("Drive/BotAccDegrees", accelerationXY.getAngle().getDegrees());
    Logger.getInstance().recordOutput("Drive/Odometry", getPose2d());
 
    velocityHistory
        .removeIf(n -> (n.getTime() < clock - DriveConstants.Tip.velocityHistorySeconds));
    velocityHistory.add(new SnapshotTranslation2D(velocityXY, clock));

    if (Constants.debug) {
      botVelocityMag.setDouble(latestVelocity);
      botAccelerationMag.setDouble(latestAcceleration);
      botVelocityAngle.setDouble(velocityXY.getAngle().getDegrees());
      botAccelerationAngle.setDouble(accelerationXY.getAngle().getDegrees());
    }
  }
}
