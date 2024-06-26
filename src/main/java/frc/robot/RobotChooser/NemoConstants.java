package frc.robot.RobotChooser;

import frc.utility.OrangeMath;

public class NemoConstants implements RobotChooserInterface {
  // drive IDs
  public int getFrontLeftDriveID() {
    return 17;
  }

  public int getFrontRightDriveID() {
    return 18;
  }

  public int getBackRightDriveID() {
    return 19;
  }

  public int getBackLeftDriveID() {
    return 16;
  }

  // rotation IDs
  public int getFrontLeftRotationID() {
    return 21;
  }

  public int getFrontRightRotationID() {
    return 15;
  }

  public int getBackRightRotationID() {
    return 22;
  }

  public int getBackLeftRotationID() {
    return 20;
  }

  // measured distance between wheels (24 and 11/16 in)
  public double getDistWheelMetersX() {
    return OrangeMath.inchesToMeters(24.6875 / 2);
  }

  public double getDistWheelMetersY() {
    return OrangeMath.inchesToMeters(24.6875 / 2);
  }

  // top speed at full motor output is 91 rot/sec with voltage comp at 11.5 volts
  // however, setting the max speed to 91 only allows us to reach 86 due to insufficent kV
  public double getMaxSpeedMetersPerSec() { // TODO
    return 3.695;
  }

  public double getMaxRotationSpeedRadPerSec() {
    return 7.854; // physical limit of the bot
  }

  public double getAutoRotatekP() {
    return 0.008;
  }

  public double getAutoRotatekD() {
    return 0.0004;
  }

  // For tuning, graph Duty Cycle Position in the REV Hardware Client
  public double getRotationkP() {
    return 0.009;
  }

  public double getRotationkD() {
    return 0.0002;
  }

  public double getDriveGearRatio() {
    return 7.80; // drive gear ratio
  }

  public double getRotationGearRatio() {
    return 1.0; // one to one
  }

  public double getDrivekSVolts() {
    return 0.182;
  }

  // Feed Forward parameters for Drive PID
  public double[] getDriveffSpeedMetersPerSecThresholds() {
    // define speed at which each voltage value will be used
    double[] feedForwardMetersPerSecThreshold = new double[4];
    // values must be in ascending order
    feedForwardMetersPerSecThreshold[0] = 0.0; // Must be zero
    feedForwardMetersPerSecThreshold[1] = 1.7;
    feedForwardMetersPerSecThreshold[2] = 2.6;
    feedForwardMetersPerSecThreshold[3] = 3.18;
    return feedForwardMetersPerSecThreshold;
  }

  public double[] getDriveffVoltsOverMetersPerSec() {
    double[] voltsOverMetersPerSecAtSpeedThresholds = new double[4];
    voltsOverMetersPerSecAtSpeedThresholds[0] = 3.3;
    voltsOverMetersPerSecAtSpeedThresholds[1] = 3.3;
    voltsOverMetersPerSecAtSpeedThresholds[2] = 3.3;
    voltsOverMetersPerSecAtSpeedThresholds[3] = 3.37;
    return voltsOverMetersPerSecAtSpeedThresholds;
  }

  public double getAutoTrajectoryXYkP() {
    return 0.1;
  }

  public double getAutoTrajectoryXYkI() {
    return 0.0;
  }

  public double getAutoTrajectoryXYkD() {
    return 0.0;
  }

  public double getAutoTrajectoryXYkiZ() {
    return 0.0;
  }

  public double getAutoTrajectoryRotkP() {
    return 2.0;
  }

  public double getAutoTrajectoryRotkI() {
    return 0.0;
  }

  public double getAutoTrajectoryRotkD() {
    return 0.01;
  }

  public double getAutoTrajectoryRotkiZ() {
    return 0.0;
  }

  @Override
  public double getMinAutoRotateStoppedPower() {
    return 0.003;  // TODO
  }

  @Override
  public double getMinAutoRotateSlowPower() {
    return 0.01;  // TODO
  }

  @Override
  public double getMinAutoRotateFastPower() {
    return 0.03;  // TODO
  }
}
