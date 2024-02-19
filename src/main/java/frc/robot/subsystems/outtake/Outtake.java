package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private OuttakeIO io;
  private OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private double targetRPS;

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
  }

  public double getTargetRPS() {
    return targetRPS;
  }

  public void periodic() {
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
}
