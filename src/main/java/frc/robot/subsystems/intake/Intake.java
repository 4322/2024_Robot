package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeConfig;
import frc.robot.commands.AutoAcquireNote;
import frc.robot.commands.XboxControllerRumble;
import frc.robot.subsystems.RobotCoordinator;
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
  private boolean isFeeding;

  private AutoAcquireNote autoAcquireNote = new AutoAcquireNote();
  private XboxControllerRumble xBoxRumble = new XboxControllerRumble();

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
    existenceTimer = new Timer();
  }

  @Override
  public void periodic() {
    RobotCoordinator coordinator = RobotCoordinator.getInstance();
    // initialize motor internal encoder position until the intake isn't moving
    if (Constants.intakeEnabled && !initialized && !existenceTimer.hasElapsed(5) 
          && coordinator.getInitAbsEncoderPressed()) {
      existenceTimer.start();
      initialized = io.initMotorPos();
    }
    if (Constants.intakeEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeConstants.Logging.key, inputs);

      switch (intakeState) {
        case retracted:
          if (coordinator.getIntakeButtonPressed()
              && (coordinator.onOurSideOfField() || !coordinator.noteInRobot())) {
            intakeState = IntakeStates.deploying;
          }
          break;
        case deploying:
          if (coordinator.canDeploy()) {
            deploy();
          }
          if (!coordinator.getIntakeButtonPressed()) {
            intakeState = IntakeStates.retracting;
          } else if (isDeployed() && !coordinator.noteInRobot()) {
            intakeState = IntakeStates.feeding;
          } else if (coordinator.noteInRobot() && isDeployed()) {
            intakeState = IntakeStates.notePastIntake;
          }
          break;
        case feeding:
          if (coordinator.isIntakeDeployed()) {
            intake();
          }
          if (!coordinator.getIntakeButtonPressed()) {
            intakeState = IntakeStates.retracting;
          } else if (coordinator.noteInIntake()) {
            intakeState = IntakeStates.noteObtained;
          } else if (coordinator.getAutoIntakeButtonPressed() && coordinator.noteInVision()) {
            if (!autoAcquireNote.isScheduled()) {
              CommandScheduler.getInstance().schedule(autoAcquireNote);
            }
          }
          break;
        case noteObtained:
          if (coordinator.isIntakeDeployed()) {
            intake();
          }
          if (!coordinator.noteInIntake()) {
            CommandScheduler.getInstance().schedule(xBoxRumble);
            intakeState = IntakeStates.notePastIntake;
          }
          break;
        case notePastIntake:
          stopFeeder();
          if (!coordinator.onOurSideOfField() || !coordinator.getIntakeButtonPressed()) {
            intakeState = IntakeStates.retracting;
          } else if (!coordinator.noteInRobot()) {
            intakeState = IntakeStates.feeding;
          }
          break;
        case retracting:
          stopFeeder();
          if (coordinator.canRetract()) {
            retract();
          }
          if (coordinator.getIntakeButtonPressed()
              && (coordinator.onOurSideOfField() || !coordinator.noteInRobot())) {
            intakeState = IntakeStates.deploying;
          } else if (coordinator.isIntakeRetracted()) {
            intakeState = IntakeStates.retracted;
          }
          break;
      }
    }
  }

  public void intake() {
    if (Constants.intakeEnabled && initialized) {
      io.setFeedingVoltage(IntakeConstants.IntakeConfig.intakeFeedVoltage);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeStopped", false);
      isFeeding = true;
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled && initialized) {
      io.setFeedingVoltage(IntakeConstants.IntakeConfig.intakeEjectVoltage);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeStopped", false);
      isFeeding = true;
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

  public void stopFeeder() {
    if (Constants.intakeEnabled && initialized) {
      io.stopFeeder();
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", true);
      isFeeding = false;
    }
  }

  public void stopDeployer() {
    if (Constants.intakeEnabled && initialized) {
      io.stopFeeder();
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", true);
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
    if (Constants.intakeEnabled && initialized) {
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

  public boolean isFeeding() {
    return isFeeding;
  }

  public IntakeStates getState() {
    return intakeState;
  }

  public void setState(IntakeStates newState) {
    intakeState = newState;
  }
}
