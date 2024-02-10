package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class DriveTest implements DriveInterface {

  public double getAngle() {
    return 0;
  }

  public double getPitch() {
    return 0;
  }

  public double getAngularVelocity() {
    return 0;
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d();
  }

  public boolean isRobotMoving() {
    return true;
  }

  public boolean isRobotMovingFast() {
    return true;
  }

  public void resetFieldCentric() {
    return;
  }

  public void drive(double driveX, double driveY, double rotate) {
    return;
  }

  public void drive(double driveX, double driveY, double rotate, Translation2d centerOfRotation) {
    return;
  }

  public void driveAutoRotate(double driveX, double driveY, double targetDeg) {
    return;
  }

  public void resetRotatePID() {
    return;
  }

  public void updateOdometry() {
    return;
  }

  public void updateVision() {
    return;
  }

  public void updateVision(Pose2d pose, double timestampSeconds) {
    return;
  }

  public void resetOdometry(Pose2d pose) {
    return;
  }

  public void setCoastMode() {
    return;
  }

  public void setBrakeMode() {
    return;
  }

  public void stop() {
    return;
  }

  public Pose2d getPose2d() {
    return new Pose2d();
  }

  public void setModuleStates() {
    return;
  }

  // this might be bad
  public SwerveModulePosition[] getModulePostitions() {
    return new SwerveModulePosition[4];
  }

  // this too
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(new Translation2d[4]);
  }

  public boolean isPseudoAutoRotateEnabled() {
    return Constants.psuedoAutoRotateEnabled;
  }

  public double getMaxManualRotationEntry() {
    return Constants.DriveConstants.Manual.maxManualRotation;
  }

  public String getInputScaling() {
    return Constants.driveInputScaling;
  }

  public String getControlType() {
    return Constants.controllerType;
  }
}
