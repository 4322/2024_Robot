package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private OuttakeIO io;
  private OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private double topTargetRPS;
  private double bottomTargetRPS;
  private double pivotTarget;
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
        pivot(inputs.targetPivotPosition);
      }
    }
  }

  public void outtake(double topTargetRPS, double bottomTargetRPS) {
    if (Constants.outtakeEnabled && io.pivotIsInitialized()) {
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

  public void pivot(double rotations) {
    if (Constants.outtakePivotEnabled && io.pivotIsInitialized()) {
      // Overrides operator shooting presets
      if (Constants.outtakeTuningMode && inputs.tuneOuttakeOverrideEnable) {
        rotations = inputs.targetPivotPosition;
      }
      
      // Code that limits forward movement of shooter if requested
      if (rotations > Constants.OuttakeConstants.forwardSoftLimitThresholdRotations) {
        rotations = Constants.OuttakeConstants.forwardSoftLimitThresholdRotations;
      }
      io.setPivotTarget(rotations);
      pivotTarget = rotations;
      Logger.recordOutput("Outtake/PivotTargetRotations", rotations);
      Logger.recordOutput("Outtake/PivotStopped", false);
    }
  }

  public void resetPivot() {
    if (Constants.outtakePivotEnabled && io.pivotIsInitialized()) {
      io.setPivotTarget(Constants.OuttakeConstants.defaultPivotPositionRotations);
      pivotTarget = Constants.OuttakeConstants.defaultPivotPositionRotations;
      Logger.recordOutput(
          "Outtake/PivotTargetRotations", Constants.OuttakeConstants.defaultPivotPositionRotations);
      Logger.recordOutput("Outtake/PivotStopped", false);
    }
  }

  public void stopPivot() {
    if (Constants.outtakePivotEnabled && io.pivotIsInitialized()) {
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
    }
  }

  public void setPivotBrakeMode() {
    if (Constants.outtakePivotEnabled) {
      isInCoast = false;
      io.setPivotBrakeMode();
    }
  }

  public void setFlywheelCoastMode() {
    if (Constants.outtakePivotEnabled) {
      io.setFlywheelCoastMode();
    }
  }

  public void setFlywheelBrakeMode() {
    if (Constants.outtakePivotEnabled) {
      io.setFlywheelBrakeMode();
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
    return io.pivotIsInitialized();
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

  public boolean inShotTuningMode() {
    return inputs.tuneOuttakeOverrideEnable;
  }
}
