package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private OuttakeIO io;
  private OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private double targetRPM;

  public Outtake() {
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
  }

  public double getTargetRPM(){ return targetRPM; }

  public void periodic() {
    if (Constants.outtakeEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs("Outtake", inputs);
      Logger.recordOutput("Outtake/TopRotationsPerSecAbs", Math.abs(inputs.topRotationsPerSec));
      Logger.recordOutput(
          "Outtake/BottomRotationsPerSecAbs", Math.abs(inputs.bottomRotationsPerSec));
    }
  }

  public void outtake(double targetRPM) {
    if (Constants.outtakeEnabled) {
      this.targetRPM = targetRPM;
      io.setOuttakeRPM(
          Constants.OuttakeConstants.topOuttakeRPM, Constants.OuttakeConstants.bottomOuttakeRPM);
      Logger.recordOutput(
          "Outtake/TopOuttakeTargetSpeedRPM", Constants.OuttakeConstants.topOuttakeRPM);
      Logger.recordOutput(
          "Outtake/BottomOuttakeTargetSpeedRPM", Constants.OuttakeConstants.bottomOuttakeRPM);
      Logger.recordOutput("Outtake/OuttakeStopped", false);
    }
  }

  public void stopOuttake() {
    if (Constants.outtakeEnabled) {
      io.stopOuttake();
      Logger.recordOutput("Outtake/TopOuttakeTargetSpeedRPM", 0);
      Logger.recordOutput("Outtake/BottomOuttakeTargetSpeedRPM", 0);
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
            inputs.topRotationsPerSec * 60, targetRPM, OuttakeConstants.outtakeToleranceRPM)
        && OrangeMath.equalToEpsilon(
            inputs.bottomRotationsPerSec * 60, targetRPM, OuttakeConstants.outtakeToleranceRPM));
  }
}
