package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.utility.OrangeMath;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Timer existenceTimer;
  private boolean deployInitialized;
  private double deployTarget = 99999; // set to very high value in case target not yet set
  private boolean isFeeding;
  private boolean deployRequested;

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
    // Check if encoders have already been initialized after power cycle
    // If so, we don't need to reinitialize
    if (OrangeMath.equalToEpsilon(
        inputs.heliumRelativeRotations,
        Constants.EncoderInitializeConstants.initializedRotationsFlag,
        Constants.EncoderInitializeConstants.initializedRotationsTolerance)) {
      deployInitialized = true;
    }

    // initialize motor internal encoder position until the intake isn't moving
    if (Constants.intakeDeployerEnabled
        && !deployInitialized
        && !existenceTimer.hasElapsed(5)
        && coordinator.getInitAbsEncoderPressed()) {
      existenceTimer.start();
      deployInitialized = io.initMotorPos();
    }
    if (Constants.intakeEnabled || Constants.intakeDeployerEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeConstants.Logging.key, inputs);
    }
  }

  public void intake() {
    if (Constants.intakeEnabled && deployInitialized) {
      io.setFeedingVoltage(inputs.intakeFeederVoltage);
      Logger.recordOutput(IntakeConstants.Logging.feederKey + "IntakeStopped", false);
      isFeeding = true;
    }
  }

  public void outtake() {
    if (Constants.intakeEnabled && deployInitialized) {
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
    if (Constants.intakeEnabled && deployInitialized) {
      io.stopFeeder();
      Logger.recordOutput(IntakeConstants.Logging.key + "IntakeStopped", true);
      isFeeding = false;
    }
  }

  public void stopDeployer() {
    if (Constants.intakeDeployerEnabled && deployInitialized) {
      io.stopFeeder();
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", true);
      deployRequested = false;
    }
  }

  public void deploy() {
    if (Constants.intakeDeployerEnabled && deployInitialized) {
      io.setDeployTarget(inputs.deployPositionRotations);
      deployTarget = inputs.deployPositionRotations;
      Logger.recordOutput(
          IntakeConstants.Logging.deployerKey + "DeployTargetRotations",
          inputs.deployPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", false);
      deployRequested = true;
    }
  }

  public void retract() {
    if (Constants.intakeDeployerEnabled && deployInitialized) {
      io.setDeployTarget(inputs.retractPositionRotations);
      deployTarget = inputs.retractPositionRotations;
      Logger.recordOutput(
          IntakeConstants.Logging.deployerKey + "DeployTargetRotations",
          inputs.retractPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", false);
      deployRequested = false;
    }
  }

  public boolean isAtPosition() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations, deployTarget, IntakeConstants.Deploy.toleranceRotations);
  }

  public boolean isDeployed() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations,
        inputs.deployPositionRotations,
        IntakeConstants.Deploy.toleranceRotations);
  }

  public boolean isDeploying() {
    return deployRequested && !isDeployed();
  }

  public double getDeployRotations() {
    return inputs.deployRotations;
  }

  public boolean isRetracted() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations,
        inputs.retractPositionRotations,
        IntakeConstants.Deploy.toleranceRotations);
  }

  public boolean isDeployInitialized() {
    return deployInitialized;
  }

  public boolean isFeeding() {
    return isFeeding;
  }
}
