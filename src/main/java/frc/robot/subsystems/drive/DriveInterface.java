package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveInterface extends Subsystem {
  public double getAngle();

  public double getPitch();

  public double getAngularVelocity();

  public Rotation2d getRotation2d();

  public boolean isRobotMoving();

  public boolean isRobotMovingFast();

  public void resetFieldCentric();

  public void drive(double driveX, double driveY, double rotate);

  public void drive(double driveX, double driveY, double rotate, Translation2d centerOfRotation);

  public void driveAutoRotate(double driveX, double driveY, double targetDeg);

  public void resetRotatePID();

  public void updateOdometry();

  public void updateVision(Pose2d pose, double timestampSeconds);

  public void resetOdometry(Pose2d pose);

  public void setCoastMode();

  public void setBrakeMode();

  public void stop();

  public Pose2d getPose2d();

  public SwerveModulePosition[] getModulePostitions();

  public SwerveDriveKinematics getKinematics();

  public boolean isPseudoAutoRotateEnabled();

  public double getMaxManualRotationEntry();

  public String getInputScaling();

  public String getControlType();
}
