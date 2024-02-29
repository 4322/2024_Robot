package frc.robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  TrapezoidProfile.State m_goal;
  ShuffleboardTab tab;
  GenericEntry kP;
  GenericEntry slowPos;
  GenericEntry deployerRPS;
  private boolean isFeeding;
  private boolean deployRequested;
  double deployVolts;
  private static Intake intake;
  public enum IntakeDeployState {
    Unknown,
    Deployed,
    Deploying,
    Retracting,
    Retracted
  }
  IntakeDeployState state;
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

    if (io == null) {
      io = new IntakeIO() {};
    }
    existenceTimer = new Timer();
    if(Constants.debug)
    {
      tab = Shuffleboard.getTab("intake");
      kP = tab.add("deployer kP",Constants.IntakeConstants.deployKp)
        .withPosition(3, 1)
        .withSize(1, 1).getEntry();
      slowPos = tab.add("deployer slowing position", Constants.IntakeConstants.slowPos)
        .withPosition(0,2)
        .withSize(1,1).getEntry();
      deployerRPS = tab.add("deployer RPS",0)
        .withPosition(1, 2)
        .withSize(1, 1).getEntry();
    }
  }

  @Override
  public void periodic() {
    RobotCoordinator coordinator = RobotCoordinator.getInstance();

    if (Constants.intakeEnabled || Constants.intakeDeployerEnabled) {
      io.updateInputs(inputs);
      Logger.processInputs(IntakeConstants.Logging.key, inputs);
    }
    if(Constants.intakeDeployerEnabled)
    {
      switch(state)
      {
        case Unknown:
          break;
        case Deploying:
          if(inputs.heliumRotations < slowPos.getDouble(IntakeConstants.slowPos))
          {
            deployVolts = IntakeConstants.Deploy.slowDeployVolts;
          }
          else
          {
            deployVolts = IntakeConstants.Deploy.fastDeployVolts;
          }
          if(isDeployed())
          {
            state = IntakeDeployState.Deployed;
          }
          break;
        case Retracting:
          
          if(isRetracted())
          {
            setDeployerBrakeMode();
            state = IntakeDeployState.Retracted;
          }
          break;
        case Deployed:
          deployVolts = 0;
          break;
        case Retracted:
          if(!OrangeMath.equalToEpsilon(inputs.deployRotations, IntakeConstants.Deploy.retractTargetPosition, IntakeConstants.Deploy.retractTolerance))
          { 
            state = IntakeDeployState.Retracting;
          }
          break;
        default:
          break;
      }
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
      io.stopDeployer();
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", true);
      deployRequested = false;
    }
  }

  public void deploy() {
    if (Constants.intakeDeployerEnabled && deployInitialized) {
      if(state != IntakeDeployState.Deploying)
      {
        setDeployerCoastMode();
      }
      state = IntakeDeployState.Deploying;
      io.setDeployVoltage(deployVolts);
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
      if(state != IntakeDeployState.Retracting)
      {
        setDeployerCoastMode();
      }
      state = IntakeDeployState.Retracting;
      io.setDeployVoltage(deployVolts);
      deployTarget = inputs.retractPositionRotations;
      Logger.recordOutput(
          IntakeConstants.Logging.deployerKey + "DeployTargetRotations",
          inputs.retractPositionRotations);
      Logger.recordOutput(IntakeConstants.Logging.deployerKey + "DeployStopped", false);
      deployRequested = false;
    }
  }


  public boolean isDeployed() {
    return OrangeMath.equalToEpsilon(
        inputs.deployRotations,
        IntakeConstants.Deploy.deployTargetPosition,
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
