package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.DeployConfig;
import frc.robot.commands.IntakeManual.IntakeStates;
import frc.utility.OrangeMath;
import frc.utility.OrangePIDController;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean isFeeding;
  private boolean isEjecting;
  private boolean isInCoast;
  private double desiredVolts;
  private static Intake intake;

  public enum IntakeDeployState {
    Unknown,
    Deployed,
    Deploying,
    Retracting,
    Retracted,
    Climbing,
    Climbed
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
        if (Constants.intakeEnabled || Constants.intakeDeployerEnabled) {
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
        case Climbing:
          if (inputs.heliumAbsRotations
              < (IntakeConstants.DeployConfig.climbTargetPosition - inputs.slowPos)) {
            // move pivot to climb position from deployed
            desiredVolts = DeployConfig.climbForwardVoltage;
          } else if (inputs.heliumAbsRotations
              > (IntakeConstants.DeployConfig.climbTargetPosition + inputs.slowPos)) {
            // move pivot to climb position from retracted
            desiredVolts = DeployConfig.climbReverseVoltage;
          } else if (inputs.heliumAbsRotations < IntakeConstants.DeployConfig.climbTargetPosition) {
            // retract slowly if slightly past climbing position
            desiredVolts = 
                DeployConfig.climbForwardVoltage
                    * (IntakeConstants.DeployConfig.climbTargetPosition
                        - inputs.heliumAbsRotations)
                    / inputs.slowPos;
          } else {
            // retract slowly if slightly above climbing position
            desiredVolts =
                DeployConfig.climbReverseVoltage
                    * (inputs.heliumAbsRotations
                        - IntakeConstants.DeployConfig.climbTargetPosition)
                    / inputs.slowPos;
          }
          if (isClimbFinished()) {
            state = IntakeDeployState.Climbed;
            stopDeployer();
          } else {
            io.setDeployVoltage(desiredVolts);
          }
          break;
        case Deployed:
          if (!OrangeMath.equalToEpsilon(
              inputs.heliumAbsRotations,
              IntakeConstants.DeployConfig.deployTargetPosition,
              IntakeConstants.DeployConfig
                  .correctionTolerance)) // if intake has popped up too far while driving
          {
            deploy(); // move it back into position
          }
          break;
        case Retracted:
          if (!OrangeMath.equalToEpsilon(
              inputs.heliumAbsRotations,
              IntakeConstants.DeployConfig.retractTargetPosition,
              IntakeConstants.DeployConfig
                  .correctionTolerance)) // if intake has drooped too far while driving
          {
            retract(); // move it back into position
          }
          break;
        case Climbed:
          if (!isClimbFinished()) {
            deployClimbPosition(); // move it back into position
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
      isEjecting = false;
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled) {
      io.setFeedingVoltage(inputs.intakeEjectVoltage);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "State", "Outtaking");
      isFeeding = false;
      isEjecting = true;
    }
  }

  public void setIntakeBrakeMode() {
    if (Constants.intakeEnabled) {
      io.setIntakeBrakeMode();
    }
  }

  public void setDeployerBrakeMode() {
    if (Constants.intakeDeployerEnabled) {
      isInCoast = false;
      io.setDeployerBrakeMode();
    }
  }

  public void setIntakeCoastMode() {
    if (Constants.intakeEnabled) {
      io.setIntakeCoastMode();
    }
  }

  public void setDeployerCoastMode() {
    if (Constants.intakeDeployerEnabled) {
      isInCoast = true;
      io.setDeployerCoastMode();
    }
  }

  public void stopFeeder() {
    if (Constants.intakeEnabled) {
      io.stopFeeder();
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "State", "Stopped");
      isFeeding = false;
      isEjecting = false;
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

  public void deployClimbPosition() {
    if (Constants.intakeDeployerEnabled) {
      state = IntakeDeployState.Climbing;
      desiredVolts = 0;
      
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "desiredVolts", desiredVolts);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "State", "Deploying");
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

  private boolean isClimbFinished() {
    return OrangeMath.equalToEpsilon(
        inputs.heliumAbsRotations,
        IntakeConstants.DeployConfig.climbTargetPosition,
        IntakeConstants.DeployConfig.atTargetTolerance);
  }

  public boolean isFeeding() {
    return isFeeding;
  }

  public boolean isEjecting() {
    return isEjecting;
  }

  public boolean deployInCoast() {
    return isInCoast;
  }
}
