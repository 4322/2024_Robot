package frc.robot.subsystems.intakeDeployer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeDeployerConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class IntakeDeployer extends SubsystemBase {

  public enum IntakeDeployerStates {
    deploying,
    retracting,
    deployed,
    retracted;
  }

  private IntakeDeployerIO io;
  private IntakeDeployerIOInputsAutoLogged inputs = new IntakeDeployerIOInputsAutoLogged();
  private Timer existenceTimer;
  private boolean initialized;
  private double deployTarget = 99999; // set to very high value in case target not yet set
  private IntakeDeployerStates deployerState = IntakeDeployerStates.retracted;

  private static IntakeDeployer intakeDeployer;

  public static IntakeDeployer getInstance() {
    if (intakeDeployer == null) {
      intakeDeployer = new IntakeDeployer();
    }
    return intakeDeployer;
  }

  private IntakeDeployer() {
    switch (Constants.currentMode) {
      case REAL:
        if (Constants.intakeDeployerEnabled) {
          io = new IntakeDeployerIOReal();
        }
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    if (io == null) {
      io = new IntakeDeployerIO() {};
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

    if (Constants.intakeDeployerEnabled && initialized) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeDeployerConstants.Logging.key, inputs);

      switch(deployerState) {
        case deploying:
          deploy();
          if (isDeployed()) {
            deployerState = IntakeDeployerStates.deployed;
          }
          break;
        case deployed:
          break;
        case retracting:
          retract();
          if (isRetracted()) {
            deployerState = IntakeDeployerStates.retracted;
          }
          break;
        case retracted:
          break;
      }
    }
  }

  public void deploy() {
    if (Constants.intakeEnabled && initialized) {
      io.setDeployTarget(IntakeDeployerConstants.Deploy.deployPositionRotations);
      deployTarget = IntakeDeployerConstants.Deploy.deployPositionRotations;
      Logger.recordOutput(
          IntakeDeployerConstants.Logging.key + "DeployTargetRotations",
          IntakeDeployerConstants.Deploy.deployPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployStopped", false);
    }
  }

  public void retract() {
    if (Constants.intakeDeployerEnabled && initialized) {
      io.setDeployTarget(IntakeDeployerConstants.Deploy.retractPositionRotations);
      deployTarget = IntakeDeployerConstants.Deploy.retractPositionRotations;
      Logger.recordOutput(
          IntakeDeployerConstants.Logging.key + "DeployTargetRotations",
          IntakeDeployerConstants.Deploy.retractPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployStopped", false);
    }
  }

  public boolean isAtPosition() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations, deployTarget, IntakeDeployerConstants.Deploy.toleranceRotations);
  }

  public void setBrakeMode() {
    if (Constants.intakeDeployerEnabled && initialized) {
      io.setBrakeMode();
      Logger.recordOutput(IntakeDeployerConstants.Logging.key + "TargetBrakeMode", "Brake");
    }
  }

  public void setCoastMode() {
    if (Constants.intakeDeployerEnabled && initialized) {
      io.setCoastMode();
      Logger.recordOutput(IntakeDeployerConstants.Logging.key + "TargetBrakeMode", "Coast");
    }
  }

  public void stopDeploy() {
    if (Constants.intakeDeployerEnabled && initialized) {
      io.stopDeploy();
      Logger.recordOutput(IntakeDeployerConstants.Logging.key + "DeployStopped", true);
    }
  }

  public boolean isDeployed() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations,
        IntakeDeployerConstants.Deploy.deployPositionRotations,
        IntakeDeployerConstants.Deploy.toleranceRotations);
  }

  public boolean isRetracted() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations,
        IntakeDeployerConstants.Deploy.retractPositionRotations,
        IntakeDeployerConstants.Deploy.toleranceRotations);
  }

  public boolean isInitialized() {
    return initialized;
  }

  public IntakeDeployerStates getState() {
    return deployerState;
  }

  public void setState(IntakeDeployerStates newState) {
    deployerState = newState;
  }
}
