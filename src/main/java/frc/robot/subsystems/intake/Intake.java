package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.DeployConfig;
import frc.utility.OrangeMath;
import frc.utility.OrangePIDController;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean isFeeding;
  private double desiredVolts;
  private static Intake intake;

  public enum IntakeDeployState {
    Unknown,
    Deployed,
    Deploying,
    Retracting,
    Retracted
  }

  private IntakeDeployState state;
  private OrangePIDController deployController;

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

    state = IntakeDeployState.Unknown;
    deployController = new OrangePIDController(0);

    if (io == null) {
      io = new IntakeIO() {};
    }
  }

  @Override
  public void periodic() {
    if (Constants.intakeEnabled || Constants.intakeDeployerEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeConstants.Logging.key, inputs);
    }
    if (Constants.intakeDeployerEnabled) {
      if (inputs.deployKp != deployController.getKp()) {
        deployController.setKp(inputs.deployKp);
      }
      switch (state) {
        case Unknown:
          break;
        case Deploying:
          if (inputs.heliumAbsRotations > inputs.slowPos) {
            // cruise phase
            desiredVolts = DeployConfig.peakReverseVoltage;
          } else {
            // ramp down
            desiredVolts =
                DeployConfig.peakReverseVoltage * inputs.heliumAbsRotations / inputs.slowPos;
          }
          if (isDeployFinished()) {
            state = IntakeDeployState.Deployed;
            stopDeployer();
          } else {
            io.setDeployVoltage(desiredVolts);
          }
          break;
        case Retracting:
          if (inputs.heliumAbsRotations
              < (IntakeConstants.DeployConfig.retractTargetPosition - inputs.slowPos)) {
            // cruise phase
            desiredVolts = DeployConfig.peakForwardVoltage;
          } else {
            // ramp down
            desiredVolts =
                DeployConfig.peakForwardVoltage
                    * (IntakeConstants.DeployConfig.retractTargetPosition
                        - inputs.heliumAbsRotations)
                    / inputs.slowPos;
          }
          if (isRetractFinished()) {
            state = IntakeDeployState.Retracted;
            stopDeployer();
          } else {
            io.setDeployVoltage(desiredVolts);
          }
          break;
        case Deployed:
          break;
        case Retracted:
          if (!OrangeMath.equalToEpsilon(
              inputs.heliumAbsRotations,
              IntakeConstants.DeployConfig.retractTargetPosition,
              IntakeConstants.DeployConfig
                  .deployFallTolerance)) // if intake has drooped too far while driving
          {
            retract(); // move it back into position
          }
          break;
        default:
          break;
      }
    }
  }

  public void intake() {
    if (Constants.intakeEnabled) {
      io.setFeedingVoltage(inputs.intakeFeederVoltage);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "State", "Intaking");
      isFeeding = true;
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled) {
      io.setFeedingVoltage(inputs.intakeEjectVoltage);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "State", "Outtaking");
      isFeeding = true;
    }
  }

  public void setIntakeBrakeMode() {
    if (Constants.intakeEnabled) {
      io.setIntakeBrakeMode();
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "NeutralMode", "Brake");
    }
  }

  public void setDeployerBrakeMode() {
    if (Constants.intakeDeployerEnabled) {
      io.setDeployerBrakeMode();
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "NeutralMode", "Brake");
    }
  }

  public void setIntakeCoastMode() {
    if (Constants.intakeEnabled) {
      io.setIntakeCoastMode();
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "NeutralMode", "Coast");
    }
  }

  public void setDeployerCoastMode() {
    if (Constants.intakeDeployerEnabled) {
      io.setDeployerCoastMode();
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "NeutralMode", "Coast");
    }
  }

  public void stopFeeder() {
    if (Constants.intakeEnabled) {
      io.stopFeeder();
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "State", "Stopped");
      isFeeding = false;
    }
  }

  public void stopDeployer() {
    if (Constants.intakeDeployerEnabled) {
      desiredVolts = 0;
      io.setDeployVoltage(desiredVolts);
      io.stopDeployer();
      if ((state == IntakeDeployState.Deploying) || (state == IntakeDeployState.Retracting)) {
        state = IntakeDeployState.Unknown;
      }
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "desiredVolts", desiredVolts);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "State", "Stopped");
    }
  }

  public void deploy() {
    if (Constants.intakeDeployerEnabled) {
      state = IntakeDeployState.Deploying;
      desiredVolts = 0;
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "desiredVolts", desiredVolts);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "State", "Deploying");
    }
  }

  public void retract() {
    if (Constants.intakeDeployerEnabled) {
      state = IntakeDeployState.Retracting;
      desiredVolts = 0;
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "desiredVolts", desiredVolts);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "State", "Retracting");
    }
  }

  public boolean isDeployed() {
    return (state == IntakeDeployState.Deployed) || !Constants.intakeDeployerEnabled;
  }

  private boolean isDeployFinished() {
    return OrangeMath.equalToEpsilon(
        inputs.heliumAbsRotations,
        IntakeConstants.DeployConfig.deployTargetPosition,
        IntakeConstants.DeployConfig.atTargetTolerance);
  }

  public boolean isDeploying() {
    return state == IntakeDeployState.Deploying;
  }

  public double getDeployRotations() {
    return inputs.heliumAbsRotations;
  }

  public boolean isRetracted() {
    return (state == IntakeDeployState.Retracted) && Constants.intakeDeployerEnabled;
  }

  private boolean isRetractFinished() {
    return OrangeMath.equalToEpsilon(
        inputs.heliumAbsRotations,
        IntakeConstants.DeployConfig.retractTargetPosition,
        IntakeConstants.DeployConfig.atTargetTolerance);
  }

  public boolean isFeeding() {
    return isFeeding;
  }
}
