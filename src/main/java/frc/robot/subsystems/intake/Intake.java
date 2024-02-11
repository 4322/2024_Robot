package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean initialized;

  private static Intake intake;

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  private Intake() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.intakeEnabled) {
          io = new IntakeIOReal();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (io == null) {
      io = new IntakeIO() {};
    }
  }

  @Override
  public void periodic() {
    if (Constants.intakeEnabled && initialized) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeConstants.Logging.key, inputs);
    }
  }

  public void intake() {
    if (Constants.intakeEnabled && initialized) {
      io.setIntakeRPM(IntakeConstants.Intake.intakeSpeedRPM);
      Logger.recordOutput(
          IntakeConstants.Logging.key + "IntakeTargetSpeedPct",
          IntakeConstants.Intake.intakeSpeedRPM);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", false);
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled && initialized) {
      io.setIntakeRPM(IntakeConstants.Intake.outtakeSpeedRPM);
      Logger.recordOutput(
          IntakeConstants.Logging.key + "IntakeTargetSpeedPct",
          IntakeConstants.Intake.outtakeSpeedRPM);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", false);
    }
  }

  public void setBrakeMode() {
    if (Constants.intakeEnabled && initialized) {
      io.setBrakeMode();
      Logger.recordOutput(IntakeConstants.Logging.key + "TargetBrakeMode", "Brake");
    }
  }

  public void setCoastMode() {
    if (Constants.intakeEnabled && initialized) {
      io.setCoastMode();
      Logger.recordOutput(IntakeConstants.Logging.key + "TargetBrakeMode", "Coast");
    }
  }

  public void stopIntake() {
    if (Constants.intakeEnabled && initialized) {
      io.stopIntake();
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeTargetSpeedPct", 0);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", true);
    }
  }
}
