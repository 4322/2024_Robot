package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.Constants.IntakeConstants;
import frc.utility.OrangeMath;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum IntakeStates {
    retracted,
    deploying,
    feeding,
    noteObtained,
    notePastIntake,
    retracting;
  }

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeStates intakeState = IntakeStates.retracted;
  private Timer existenceTimer;
  private boolean initialized;
  private double deployTarget = 99999; // set to very high value in case target not yet set
  private RobotCoordinator coordinator;

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
    coordinator = RobotCoordinator.getInstance();

    if (io == null) {
      io = new IntakeIO() {};
    }
    existenceTimer = new Timer();
    existenceTimer.start();
  }

  @Override
  public void periodic() {
    // initialize motor internal encoder position until the intake isn't moving
    if (Constants.intakeDeployerEnabled && !initialized && !existenceTimer.hasElapsed(5)) {
      initialized = io.initMotorPos();
    }

    if (Constants.intakeEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeConstants.Logging.key, inputs);

      switch (intakeState) {
        case retracted:
          break;
        case deploying:
          break;
        case feeding:
          break;
        case noteObtained:
          break;
        case notePastIntake:
          break;
        case retracting:
          break;
      }
    }
  }

  public void intake() {
    if (Constants.intakeEnabled && initialized) {
      io.setIntakeRPM(IntakeConstants.Intake.intakeSpeedRPM);
      Logger.recordOutput(
          IntakeConstants.Logging.feederKey + "IntakeTargetSpeedPct",
          IntakeConstants.Intake.intakeSpeedRPM);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeStopped", false);
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled && initialized) {
      io.setIntakeRPM(IntakeConstants.Intake.outtakeSpeedRPM);
      Logger.recordOutput(
          IntakeConstants.Logging.feederKey + "IntakeTargetSpeedPct",
          IntakeConstants.Intake.outtakeSpeedRPM);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeStopped", false);
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
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeTargetSpeedPct", 0);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", true);
    }
  }

  public void deploy() {
    if (Constants.intakeEnabled && initialized) {
      io.setDeployTarget(IntakeConstants.Deploy.deployPositionRotations);
      deployTarget = IntakeConstants.Deploy.deployPositionRotations;
      Logger.recordOutput(
          IntakeConstants.Logging.deployerKey + "DeployTargetRotations",
          IntakeConstants.Deploy.deployPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", false);
    }
  }

  public void retract() {
    if (Constants.intakeDeployerEnabled && initialized) {
      io.setDeployTarget(IntakeConstants.Deploy.retractPositionRotations);
      deployTarget = IntakeConstants.Deploy.retractPositionRotations;
      Logger.recordOutput(
          IntakeConstants.Logging.deployerKey + "DeployTargetRotations",
          IntakeConstants.Deploy.retractPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", false);
    }
  }

  public boolean isAtPosition() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations, deployTarget, IntakeConstants.Deploy.toleranceRotations);
  }

  public boolean isDeployed() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations,
        IntakeConstants.Deploy.deployPositionRotations,
        IntakeConstants.Deploy.toleranceRotations);
  }

  public boolean isRetracted() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations,
        IntakeConstants.Deploy.retractPositionRotations,
        IntakeConstants.Deploy.toleranceRotations);
  }

  public boolean isInitialized() {
    return initialized;
  }

  public IntakeStates getState() {
    return intakeState;
  }

  public void setState(IntakeStates newState) {
    intakeState = newState;
  }
}
