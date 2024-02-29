package frc.robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.utility.OrangeMath;
import frc.utility.OrangePIDController;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  TrapezoidProfile.State m_goal;
  ShuffleboardTab tab;

  private boolean isFeeding;
  private double deployVolts;
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
    if (Constants.debug) {
      tab = Shuffleboard.getTab("intake");
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
          if (inputs.heliumAbsRotations < inputs.slowPos) {
            deployVolts = deployController.calculate(deployVolts, inputs.deployRotationsPerSec, inputs.deployMaxRotationsPerSec);
          }
          if (isDeployed()) {
            setDeployerBrakeMode();
            state = IntakeDeployState.Deployed;
          }
          break;
        case Retracting:
          if (inputs.heliumAbsRotations
              > (IntakeConstants.Deploy.retractTargetPosition - inputs.slowPos)) {
            deployVolts = deployController.calculate(deployVolts, inputs.deployRotationsPerSec, inputs.deployMaxRotationsPerSec);
          }
          if (isRetracted()) {
            setDeployerBrakeMode();
            state = IntakeDeployState.Retracted;
          }
          break;
        case Deployed:
          deployVolts = 0;
          break;
        case Retracted:
          if (!OrangeMath.equalToEpsilon(
              inputs.heliumAbsRotations,
              IntakeConstants.Deploy.retractTargetPosition,
              IntakeConstants.Deploy
                  .deployFallTolerance)) // if intake has drooped too far while driving
          {
            setDeployerCoastMode();
            state = IntakeDeployState.Retracting; // retract so it goes up to position
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
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeStopped", false);
      isFeeding = true;
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled) {
      io.setFeedingVoltage(inputs.intakeEjectVoltage);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeStopped", false);
      isFeeding = true;
    }
  }

  public void setIntakeBrakeMode() {
    if (Constants.intakeEnabled) {
      io.setIntakeBrakeMode();
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeTargetBrakeMode", "Brake");
    }
  }

  public void setDeployerBrakeMode() {
    if (Constants.intakeDeployerEnabled) {
      io.setDeployerBrakeMode();
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployerTargetBrakeMode", "Brake");
    }
  }

  public void setIntakeCoastMode() {
    if (Constants.intakeEnabled) {
      io.setIntakeCoastMode();
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeTargetBrakeMode", "Coast");
    }
  }

  public void setDeployerCoastMode() {
    if (Constants.intakeDeployerEnabled) {
      io.setDeployerCoastMode();
      Logger.recordOutput(IntakeConstants.Logging.key + "DeployerTargetBrakeMode", "Coast");
    }
  }

  public void stopFeeder() {
    if (Constants.intakeEnabled) {
      io.stopFeeder();
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", true);
      isFeeding = false;
    }
  }

  public void stopDeployer() {
    if (Constants.intakeDeployerEnabled) {
      io.stopDeployer();
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", true);
      if (isDeployed()) {
        state = IntakeDeployState.Deployed;
      } else if (isRetracted()) {
        state = IntakeDeployState.Retracted;
      } else {
        state = IntakeDeployState.Unknown;
        deployVolts = 0;
      }
    }
  }

  public void deploy() {
    if (Constants.intakeDeployerEnabled) {
      if (state != IntakeDeployState.Deploying) {
        setDeployerCoastMode();
      }
      state = IntakeDeployState.Deploying;
      io.setDeployVoltage(deployVolts);
      Logger.recordOutput(
          IntakeConstants.Logging.deployerKey + "DeployTargetRotations",
          IntakeConstants.Deploy.deployTargetPosition);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", false);
    }
  }

  public void retract() {
    if (Constants.intakeDeployerEnabled) {
      if (state != IntakeDeployState.Retracting) {
        setDeployerCoastMode();
      }
      state = IntakeDeployState.Retracting;
      io.setDeployVoltage(deployVolts);
      Logger.recordOutput(
          IntakeConstants.Logging.deployerKey + "DeployTargetRotations",
          IntakeConstants.Deploy.retractTargetPosition);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", false);
    }
  }

  public boolean isDeployed() {
    return OrangeMath.equalToEpsilon(
        inputs.heliumAbsRotations,
        IntakeConstants.Deploy.deployTargetPosition,
        IntakeConstants.Deploy.atTargetTolerance);
  }

  public boolean isDeploying() {
    return state == IntakeDeployState.Deploying;
  }

  public double getDeployRotations() {
    return inputs.heliumAbsRotations;
  }

  public boolean isRetracted() {
    return OrangeMath.equalToEpsilon(
        inputs.heliumAbsRotations,
        IntakeConstants.Deploy.retractTargetPosition,
        IntakeConstants.Deploy.atTargetTolerance);
  }

  public boolean isFeeding() {
    return isFeeding;
  }
}
