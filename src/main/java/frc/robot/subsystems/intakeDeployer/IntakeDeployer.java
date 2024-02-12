package frc.robot.subsystems.intakeDeployer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeDeployerConstants;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class IntakeDeployer extends SubsystemBase {

  private IntakeDeployerIO io;
  private IntakeDeployerIOInputsAutoLogged inputs = new IntakeDeployerIOInputsAutoLogged();
  private Timer existenceTimer;
  private boolean initialized;
  private double deployTarget = 99999; // set to very high value in case target not yet set

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
    return this.isAtPosition();
  }

  public boolean isInitialized() {
    return initialized;
  }
}
