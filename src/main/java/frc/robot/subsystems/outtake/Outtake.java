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
  private double topTargetRPS;
  private double bottomTargetRPS;
  private Timer existenceTimer;
  private double pivotTarget;
  private boolean pivotInitialized;
  private boolean isInCoast;

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
        if (Constants.outtakeEnabled || Constants.outtakePivotEnabled) {
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

  public double getTopTargetRPS() {
    return topTargetRPS;
  }

  public double getBottomTargetRPS() {
    return bottomTargetRPS;
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
      Logger.recordOutput("Outtake/TopRotationsPerSecAbs", Math.abs(inputs.topRotationsPerSec));
      Logger.recordOutput("Outtake/BottomRotationsPerSecAbs", Math.abs(inputs.bottomRotationsPerSec));
    }
    if (Constants.outtakeTuningMode && inputs.tuneOuttakeOverrideEnable) {
      if (Constants.outtakeEnabled) {
        outtake(inputs.topDebugTargetRPS, inputs.bottomDebugTargetRPS);
      }
      if (Constants.outtakePivotEnabled) {
        pivot(inputs.targetPivotPosition, false);
      }
    }
  }

  public void outtake(double topTargetRPS, double bottomTargetRPS) {
    if (Constants.outtakeEnabled && pivotInitialized) {
      // Overrides operator shooting presets
      if (Constants.outtakeTuningMode && inputs.tuneOuttakeOverrideEnable) {
        topTargetRPS = inputs.topDebugTargetRPS;
        bottomTargetRPS = inputs.bottomDebugTargetRPS;
      }
      if (bottomTargetRPS > OuttakeConstants.maxVelRotationsPerSec) {
        bottomTargetRPS = OuttakeConstants.maxVelRotationsPerSec;
      }
      if (topTargetRPS > OuttakeConstants.maxVelRotationsPerSec) {
        topTargetRPS = OuttakeConstants.maxVelRotationsPerSec;
      }
      this.topTargetRPS = topTargetRPS;
      this.bottomTargetRPS = bottomTargetRPS;
      io.setOuttakeRPS(this.topTargetRPS, this.bottomTargetRPS);
      Logger.recordOutput("Outtake/OuttakeTopTargetSpeedRPS", this.topTargetRPS);
      Logger.recordOutput("Outtake/OuttakeBottomTargerRPS", this.bottomTargetRPS);
      Logger.recordOutput("Outtake/OuttakeStopped", false);
    }
  }

  public void outtake(double targetRPS) {
    outtake(targetRPS, targetRPS);
  }

  public void pivot(double rotations, boolean limitForwardMotion) {
    if (Constants.outtakePivotEnabled && pivotInitialized) {
      // Code that limits forward movement of shooter if requested
      if (limitForwardMotion
          && rotations > Constants.OuttakeConstants.forwardSoftLimitThresholdRotations) {
        rotations = Constants.OuttakeConstants.forwardSoftLimitThresholdRotations;
      }
      // Overrides operator shooting presets
      if (Constants.outtakeTuningMode && inputs.tuneOuttakeOverrideEnable) {
        rotations = inputs.targetPivotPosition;
      }
      io.setPivotTarget(rotations);
      pivotTarget = rotations;
      Logger.recordOutput("Outtake/PivotTargetRotations", rotations);
      Logger.recordOutput("Outtake/PivotStopped", false);
    }
  }

  public void resetPivot() {
    if (Constants.outtakePivotEnabled && pivotInitialized) {
      io.setPivotTarget(Constants.OuttakeConstants.defaultPivotPositionRotations);
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
      isInCoast = true;
      io.setPivotCoastMode();
      Logger.recordOutput("Outtake/NeutralMode", "Coast");
    }
  }

  public void setPivotBrakeMode() {
    if (Constants.outtakePivotEnabled) {
      isInCoast = false;
      io.setPivotBrakeMode();
      Logger.recordOutput("Outtake/NeutralMode", "Brake");
    }
  }

  public boolean isFlyWheelUpToSpeed() {
    return OrangeMath.equalToEpsilon(
        inputs.bottomRotationsPerSec, -bottomTargetRPS, OuttakeConstants.outtakeToleranceRPS) && 
        OrangeMath.equalToEpsilon(
        inputs.topRotationsPerSec, topTargetRPS, OuttakeConstants.outtakeToleranceRPS);
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
    return inputs.tuneOuttakeOverrideEnable;
  }

  public boolean isOuttaking() {
    return topTargetRPS > 0 && bottomTargetRPS > 0;
  }

  public boolean isFeeding() {
    return topTargetRPS < 0 && bottomTargetRPS < 0;
  }

  public boolean pivotInCoast() {
    return isInCoast;
  }
}
