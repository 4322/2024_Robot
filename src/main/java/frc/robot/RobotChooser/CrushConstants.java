package frc.robot.RobotChooser;

import frc.utility.OrangeMath;

public class CrushConstants implements RobotChooserInterface {
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

  // measured distance between wheels / 2 for distance from center
  public double getDistWheelMetersX() {
    return OrangeMath.inchesToMeters(20.75 / 2);
  }

  public double getDistWheelMetersY() {
    return OrangeMath.inchesToMeters(20.75 / 2);
  }

  public double getMaxSpeedMetersPerSec() {
    return 5.0;
  }

  public double getMaxRotationSpeedRadPerSec() { // TODO
    return 12.2718; // physical limit of the bot
  }

  public double getAutoRotatekP() { 
    return 0.004;
  }

  public double getAutoRotatekD() { 
    return 0.0;
  }

  // For tuning, graph Duty Cycle Position in the REV Hardware Client
  public double getRotationkP() { // TODO
    return 2.0;
  }

  public double getRotationkD() { // TODO
    return 0.0;
  }

  public double getDriveGearRatio() {
    return 5.90278; // L2 drive gear ratio with 16t pinion
  }

  public double getRotationGearRatio() {
    return 150.0 / 7.0;
  }

  public double getDrivekSVolts() {
    return 0.315;
  }

  // Feed Forward parameters for Drive PID
  public double[] getDriveffSpeedMetersPerSecThresholds() { // TODO
    // define speed at which each voltage value will be used
    double[] feedForwardMetersPerSecThreshold = new double[4];
    // values must be in ascending order
    feedForwardMetersPerSecThreshold[0] = 0.0; // Must be zero
    feedForwardMetersPerSecThreshold[1] = 1.7;
    feedForwardMetersPerSecThreshold[2] = 2.6;
    feedForwardMetersPerSecThreshold[3] = 3.18;
    return feedForwardMetersPerSecThreshold;
  }

  public double[] getDriveffVoltsOverMetersPerSec() { // TODO
    double[] voltsOverMetersPerSecAtSpeedThresholds = new double[4];
    voltsOverMetersPerSecAtSpeedThresholds[0] = 2.3;
    voltsOverMetersPerSecAtSpeedThresholds[1] = 2.3;
    voltsOverMetersPerSecAtSpeedThresholds[2] = 2.3;
    voltsOverMetersPerSecAtSpeedThresholds[3] = 2.3;
    return voltsOverMetersPerSecAtSpeedThresholds;
  }

  public double getAutoTrajectoryXYkP() { // TODO
    return 0.1;
  }

  public double getAutoTrajectoryXYkI() { // TODO
    return 0.0;
  }

  public double getAutoTrajectoryXYkD() { // TODO
    return 0.0;
  }

  public double getAutoTrajectoryXYkiZ() {
    return 0.0;
  }

  public double getAutoTrajectoryRotkP() {
    return 0.0;
  }

  public double getAutoTrajectoryRotkI() { // TODO
    return 0.0;
  }

  public double getAutoTrajectoryRotkD() { // TODO
    return 0.0;
  }

  public double getAutoTrajectoryRotkiZ() { // TODO
    return 0.0;
  }

  @Override
  public double getMinAutoRotateStoppedPower() { 
    return 0.0033;
  }

  @Override
  public double getMinAutoRotateSlowPower() { 
    return 0.01;
  }

  @Override
  public double getMinAutoRotateFastPower() { 
    return 0.03;
  }
}
