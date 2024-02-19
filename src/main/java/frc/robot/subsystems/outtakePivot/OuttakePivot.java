package frc.robot.subsystems.outtakePivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class OuttakePivot extends SubsystemBase {
  private OuttakePivotIO io;
  private OuttakePivotIOInputsAutoLogged inputs = new OuttakePivotIOInputsAutoLogged();
  private Timer existenceTimer;
  private double pivotTarget;
  private boolean initialized;

  private static OuttakePivot outtakePivot;

  public static OuttakePivot getInstance() {
    if (outtakePivot == null) {
      outtakePivot = new OuttakePivot();
    }
    return outtakePivot;
  }

  private OuttakePivot() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.outtakeEnabled) {
          io = new OuttakePivotIOReal();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }
    if (io == null) {
      io = new OuttakePivotIO() {};
    }

    existenceTimer = new Timer();
    existenceTimer.start();
  }

  public double getTarget() {
    return pivotTarget;
  }

  public void periodic() {
    // initialize motor internal encoder position until the intake isn't moving
    if (Constants.outtakeEnabled && !initialized && !existenceTimer.hasElapsed(5)) {
      initialized = io.initPivot();
    }
    if (Constants.outtakeEnabled && initialized) {
      io.updateInputs(inputs);
      Logger.processInputs("OuttakePivot", inputs);
    }
  }

  public void pivot(double rotations) {
    if (Constants.outtakeEnabled && initialized) {
      io.setPivotTarget(rotations);
      pivotTarget = rotations;
      Logger.recordOutput("OuttakePivot/TargetRotations", rotations);
      Logger.recordOutput("OuttakePivot/Stopped", false);
    }
  }

  public void resetPivot() {
    if (Constants.outtakeEnabled && initialized) {
      io.setPivotTarget(Constants.OuttakeConstants.defaultPivotPosition);
      pivotTarget = Constants.OuttakeConstants.defaultPivotPosition;
      Logger.recordOutput(
          "OuttakePivot/TargetRotations", Constants.OuttakeConstants.defaultPivotPosition);
      Logger.recordOutput("OuttakePivot/Stopped", false);
    }
  }

  public void stopPivot() {
    if (Constants.outtakeEnabled && initialized) {
      io.stopPivot();
      Logger.recordOutput("OuttakePivot/Stopped", true);
    }
  }

  public void setCoastMode() {
    if (Constants.outtakeEnabled && initialized) {
      io.setCoastMode();
      Logger.recordOutput("OuttakePivot/NeutralMode", "Coast");
    }
  }

  public void setBrakeMode() {
    if (Constants.outtakeEnabled && initialized) {
      io.setBrakeMode();
      Logger.recordOutput("OuttakePivot/NeutralMode", "Brake");
    }
  }

  public boolean isAtPosition() {
    return OrangeMath.equalToEpsilon(
        inputs.pivotRotations, pivotTarget, OuttakeConstants.pivotToleranceRotations);
  }

  public boolean isInitialized() {
    return initialized;
  }
}
