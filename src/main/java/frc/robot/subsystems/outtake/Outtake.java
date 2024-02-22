package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private OuttakeIO io;
  private OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private double targetRPS;
  private Timer existenceTimer;
  private double pivotTarget;
  private boolean initialized;

  private static Outtake outtake;

  public static Outtake getInstance() {
    if (outtake == null) {
      outtake = new Outtake();
    }
    return outtake;
  }

  private Outtake() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.outtakeEnabled) {
          io = new OuttakeIOReal();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }
    if (io == null) {
      io = new OuttakeIO() {};
    }

    existenceTimer = new Timer();
  }

  public double getTargetRPS() {
    return targetRPS;
  }

  public double getPivotTarget() {
    return pivotTarget;
  }

  public void periodic() {
    // initialize motor internal encoder position until the intake isn't moving
    if (Constants.outtakePivotEnabled
        && !initialized
        && !existenceTimer.hasElapsed(5)
        && RobotCoordinator.getInstance().getInitAbsEncoderPressed()) {
      existenceTimer.start();
      initialized = io.initPivot();
    }
    if (Constants.outtakeEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs("Outtake", inputs);
      Logger.recordOutput("Outtake/TopRotationsPerSecAbs", Math.abs(inputs.topRotationsPerSec));
      Logger.recordOutput(
          "Outtake/BottomRotationsPerSecAbs", Math.abs(inputs.bottomRotationsPerSec));
    }
  }

  public void outtake(double targetRPS) {
    if (Constants.outtakeEnabled) {
      if (Constants.debug) {
        targetRPS = inputs.debugTargetRPS;
      }
      this.targetRPS = targetRPS;
      io.setOuttakeRPS(
          Constants.OuttakeConstants.topOuttakeRPS, Constants.OuttakeConstants.bottomOuttakeRPS);
      Logger.recordOutput(
          "Outtake/TopOuttakeTargetSpeedRPS", Constants.OuttakeConstants.topOuttakeRPS);
      Logger.recordOutput(
          "Outtake/BottomOuttakeTargetSpeedRPS", Constants.OuttakeConstants.bottomOuttakeRPS);
      Logger.recordOutput("Outtake/OuttakeStopped", false);
    }
  }

  public void pivot(double rotations) {
    if (Constants.outtakePivotEnabled && initialized) {
      if (Constants.debug) rotations = inputs.targetPivotPosition;
      io.setPivotTarget(rotations);
      pivotTarget = rotations;
      Logger.recordOutput("Outtake/PivotTargetRotations", rotations);
      Logger.recordOutput("Outtake/PivotStopped", false);
    }
  }

  public void resetPivot() {
    if (Constants.outtakePivotEnabled && initialized) {
      io.setPivotTarget(Constants.OuttakeConstants.defaultPivotPosition);
      pivotTarget = Constants.OuttakeConstants.defaultPivotPosition;
      Logger.recordOutput(
          "Outtake/PivotTargetRotations", Constants.OuttakeConstants.defaultPivotPosition);
      Logger.recordOutput("Outtake/PivotStopped", false);
    }
  }

  public void stopPivot() {
    if (Constants.outtakePivotEnabled && initialized) {
      io.stopPivot();
      Logger.recordOutput("Outtake/PivotStopped", true);
    }
  }


  public void stopOuttake() {
    if (Constants.outtakeEnabled) {
      io.stopOuttake();
      Logger.recordOutput("Outtake/TopOuttakeTargetSpeedRPS", 0);
      Logger.recordOutput("Outtake/BottomOuttakeTargetSpeedRPS", 0);
      Logger.recordOutput("Outtake/OuttakeStopped", true);
    }
  }

  public void setCoastMode() {
    if (Constants.outtakeEnabled) {
      io.setCoastMode();
      Logger.recordOutput("Outtake/NeutralMode", "Coast");
    }
  }

  public void setBrakeMode() {
    if (Constants.outtakeEnabled) {
      io.setBrakeMode();
      Logger.recordOutput("Outtake/NeutralMode", "Brake");
    }
  }

  public boolean isFlyWheelUpToSpeed() {
    return (OrangeMath.equalToEpsilon(
            inputs.topRotationsPerSec, targetRPS, OuttakeConstants.outtakeToleranceRPS)
        && OrangeMath.equalToEpsilon(
            inputs.bottomRotationsPerSec, targetRPS, OuttakeConstants.outtakeToleranceRPS));
  }

  public boolean isAtPosition() {
    return OrangeMath.equalToEpsilon(
        inputs.pivotRotations, pivotTarget, OuttakeConstants.pivotToleranceRotations);
  }

  public boolean isInitialized() {
    return initialized;
  }
}
