package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.utility.OrangeMath;

public class Intake extends SubsystemBase implements IntakeInterface {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Timer existenceTimer;
  private boolean initialized;
  private double deployTarget = 99999; // set to very high value in case target not yet set

  public Intake() {
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
      io = new IntakeIO() {
      };
    }

    existenceTimer = new Timer();
    existenceTimer.start();
  }

  @Override
  public void periodic() {
    // initialize motor internal encoder position until the intake isn't moving
    if (Constants.intakeEnabled && !initialized && !existenceTimer.hasElapsed(5)) {
      initialized = io.initMotorPos();
    }

    if (Constants.intakeEnabled && initialized) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeConstants.Logging.key, inputs);
    }
  }

  public void deploy() {
    if (Constants.intakeEnabled && initialized) {
      io.setDeployTarget(IntakeConstants.Deploy.deployPositionRotations);
      deployTarget = IntakeConstants.Deploy.deployPositionRotations;
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployTargetRotations",
          IntakeConstants.Deploy.deployPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployStopped", false);
    }
  }

  public void undeploy() {
    if (Constants.intakeEnabled && initialized) {
      io.setDeployTarget(IntakeConstants.Deploy.undeployPositionRotations);
      deployTarget = IntakeConstants.Deploy.undeployPositionRotations;
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployTargetRotations",
          IntakeConstants.Deploy.undeployPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployStopped", false);
    }
  }

  public void intake() {
    if (Constants.intakeEnabled && initialized) {
      io.setIntake(IntakeConstants.Intake.intakeSpeedPct);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeTargetSpeedPct", IntakeConstants.Intake.intakeSpeedPct);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", false);
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled && initialized) {
      io.setIntake(IntakeConstants.Intake.outtakeSpeedPct);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeTargetSpeedPct", IntakeConstants.Intake.outtakeSpeedPct);
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", false);
    }
  }

  public boolean isAtPosition() {
    return OrangeMath.equalToEpsilon(inputs.deployRotations, deployTarget, IntakeConstants.Deploy.toleranceRotations);
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

  public void stopDeploy() {
    if (Constants.intakeEnabled && initialized) {
      io.stopDeploy();
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployStopped", true);
    }
  }

}
