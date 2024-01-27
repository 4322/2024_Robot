package frc.robot.subsystems.drive.RobotChooser;

public interface RobotChooserInterface {

  public int getFrontLeftDriveID();

  public int getFrontRightDriveID();

  public int getBackRightDriveID();

  public int getBackLeftDriveID();

  public int getFrontLeftRotationID();

  public int getFrontRightRotationID();

  public int getBackRightRotationID();

  public int getBackLeftRotationID();

  public double getDistWheelMetersX();

  public double getDistWheelMetersY();

  public double getMaxSpeedMetersPerSec();

  public double getMaxRotationSpeedRadPerSec();

  public double getAutoRotatekP();

  public double getAutoRotatekD();

  public double getRotationkP();

  public double getRotationkD();

  public double getGearRatio();

  public double getDrivekSVolts();

  public double[] getDriveffSpeedMetersPerSecThresholds();

  public double[] getDriveffVoltsOverMetersPerSec();

  public double getAutoTrajectoryXYkP();

  public double getAutoTrajectoryXYkI();

  public double getAutoTrajectoryXYkD();

  public double getAutoTrajectoryRotkP();

  public double getAutoTrajectoryRotkI();

  public double getAutoTrajectoryRotkD();
}
