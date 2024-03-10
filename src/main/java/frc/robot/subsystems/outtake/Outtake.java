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
  private boolean pivotInitialized;

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
    // Check if encoders have already been initialized after power cycle
    // If so, we don't need to reinitialize
    if (Constants.outtakePivotEnabled
        && !pivotInitialized
        && OrangeMath.equalToEpsilon(
            inputs.heliumRelativeRotations,
            Constants.EncoderInitializeConstants.initializedRotationsFlag,
            Constants.EncoderInitializeConstants.initializedRotationsTolerance)) {
      pivotInitialized = true;
    }

    // initialize motor internal encoder position until the intake isn't moving
    if (Constants.outtakePivotEnabled
        && !pivotInitialized
        && !existenceTimer.hasElapsed(5)
        && RobotCoordinator.getInstance().getInitAbsEncoderPressed()) {
      existenceTimer.start();
      pivotInitialized = io.initPivot();
    }
    if ((Constants.outtakeEnabled) || (Constants.outtakePivotEnabled)) {
      io.updateInputs(inputs);
      Logger.processInputs("Outtake", inputs);
      Logger.recordOutput("Outtake/LeftRotationsPerSecAbs", Math.abs(inputs.leftRotationsPerSec));
      Logger.recordOutput("Outtake/RightRotationsPerSecAbs", Math.abs(inputs.rightRotationsPerSec));
    }
    if (Constants.outtakeTuningMode) {
      if (inputs.debugOverrideEnable) {
        if (Constants.outtakeEnabled) {
          outtake(inputs.debugTargetRPS);
        }
      }
    }
  }

  public void outtake(double targetRPS) {
    if (Constants.outtakeEnabled) {
      if (Constants.outtakeTuningMode) {
        targetRPS = inputs.debugTargetRPS;
      }
      if (targetRPS > OuttakeConstants.maxVelRotationsPerSec) {
        targetRPS = OuttakeConstants.maxVelRotationsPerSec;
      }
      this.targetRPS = targetRPS;
      io.setOuttakeRPS(this.targetRPS, this.targetRPS);
      Logger.recordOutput("Outtake/OuttakeTargetSpeedRPS", this.targetRPS);
      Logger.recordOutput("Outtake/OuttakeStopped", false);
    }
  }

  public void pivot(double rotations, boolean limitForwardMotion) {
    if (Constants.outtakePivotEnabled && pivotInitialized) {
      if (Constants.outtakeTuningMode) {
        rotations = inputs.targetPivotPosition;
      }
      io.setPivotTarget(rotations, limitForwardMotion);
      pivotTarget = rotations;
      Logger.recordOutput("Outtake/PivotTargetRotations", rotations);
      Logger.recordOutput("Outtake/PivotStopped", false);
    }
  }

  public void resetPivot() {
    if (Constants.outtakePivotEnabled && pivotInitialized) {
      io.setPivotTarget(Constants.OuttakeConstants.defaultPivotPositionRotations, true);
      pivotTarget = Constants.OuttakeConstants.defaultPivotPositionRotations;
      Logger.recordOutput(
          "Outtake/PivotTargetRotations", Constants.OuttakeConstants.defaultPivotPositionRotations);
      Logger.recordOutput("Outtake/PivotStopped", false);
    }
  }

  public void stopPivot() {
    if (Constants.outtakePivotEnabled && pivotInitialized) {
      io.stopPivot();
      Logger.recordOutput("Outtake/PivotStopped", true);
    }
  }

  public void stopOuttake() {
    if (Constants.outtakeEnabled) {
      io.stopOuttake();
      Logger.recordOutput("Outtake/OuttakeTargetSpeedRPS", 0.0);
      Logger.recordOutput("Outtake/OuttakeStopped", true);
    }
  }

  public void setPivotCoastMode() {
    if (Constants.outtakePivotEnabled) {
      io.setPivotCoastMode();
      Logger.recordOutput("Outtake/NeutralMode", "Coast");
    }
  }

  public void setPivotBrakeMode() {
    if (Constants.outtakePivotEnabled) {
      io.setPivotBrakeMode();
      Logger.recordOutput("Outtake/NeutralMode", "Brake");
    }
  }

  public boolean isFlyWheelUpToSpeed() {
    return OrangeMath.equalToEpsilon(
        inputs.rightRotationsPerSec, targetRPS, OuttakeConstants.outtakeToleranceRPS);
  }

  public boolean pivotIsAtPosition() {
    return OrangeMath.equalToEpsilon(
        inputs.pivotRotations, pivotTarget, OuttakeConstants.pivotToleranceRotations);
  }

  public boolean pivotIsInitialized() {
    return pivotInitialized;
  }

  public boolean safeToPivot() {
    return (inputs.pivotRotations > OuttakeConstants.reverseSoftLimitThresholdRotations
        && inputs.pivotRotations < OuttakeConstants.forwardSoftLimitThresholdRotations);
  }

  public boolean getDebugOverrideEnabled() {
    return inputs.debugOverrideEnable;
  }
}
