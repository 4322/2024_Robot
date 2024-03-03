package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.robot.subsystems.intake.Intake;
import frc.utility.OrangeMath;

public class IntakeManual extends Command {
  public enum IntakeStates {
    retracted,
    deploying,
    feeding,
    noteObtained,
    notePastIntake,
    retracting, preClimb, Climb;
  }

  private static IntakeStates intakeState = IntakeStates.retracted;
  ;
  private Intake intake;
  private AutoAcquireNote autoAcquireNote;
  private XboxControllerRumble xBoxRumble;

  public IntakeManual() {
    intake = Intake.getInstance();
    autoAcquireNote = new AutoAcquireNote();
    xBoxRumble = new XboxControllerRumble();

    addRequirements(intake);
  }

  @Override
  public void execute() {
    RobotCoordinator coordinator = RobotCoordinator.getInstance();
    if(!coordinator.isClimbing())
    {
      switch (intakeState) {
        case retracted:
          if(coordinator.isClimbing())
          {
            intake.stopFeeder();
            intakeState = IntakeStates.preClimb;
          }
            
          if (coordinator.getIntakeButtonPressed()
              && (!coordinator.noteInRobot())) {
            intakeState = IntakeStates.deploying;
          }
          break;
        case deploying:
          if(coordinator.isClimbing())
          {
            intake.stopFeeder();
            intakeState = IntakeStates.preClimb;
          }
          if (coordinator.canDeploy()) {
            intake.deploy();
          }
          if (!coordinator.getIntakeButtonPressed()) {
            intakeState = IntakeStates.retracting;
          } else if (coordinator.intakeIsDeployed() && !coordinator.noteInRobot()) {
            intakeState = IntakeStates.feeding;
          } else if (coordinator.noteInRobot() && coordinator.intakeIsDeployed()) {
            intakeState = IntakeStates.notePastIntake;
          }
          break;
        case feeding:
          if(coordinator.isClimbing())
          {
            intake.stopFeeder();
            intakeState = IntakeStates.preClimb;
          }
          if (coordinator.isIntakeDeployed()) {
            intake.intake();
          }
          if (!coordinator.getIntakeButtonPressed()) {
            intakeState = IntakeStates.retracting;
          } else if (coordinator.noteEnteringIntake()) {
            intakeState = IntakeStates.noteObtained;
          } else if (Constants.autoAcquireNoteEnabled
              && coordinator.getAutoIntakeButtonPressed()
              && coordinator.noteInVision()) {
            if (!autoAcquireNote.isScheduled()) {
              CommandScheduler.getInstance().schedule(autoAcquireNote);
            }
          }
          break;
        case noteObtained:
          if(coordinator.isClimbing())
          {
            intake.stopFeeder();
            intakeState = IntakeStates.preClimb;
          }
          if (coordinator.isIntakeDeployed()) {
            intake.intake();
            CommandScheduler.getInstance().schedule(xBoxRumble);
            intakeState = IntakeStates.notePastIntake;
          }
          break;
        case notePastIntake:
          if(coordinator.isClimbing())
          {
            intake.stopFeeder();
            intakeState = IntakeStates.preClimb;
          }
          intake.stopFeeder();
          if (!coordinator.getIntakeButtonPressed()) {
            intakeState = IntakeStates.retracting;
          } else if (!coordinator.noteInRobot()) {
            intakeState = IntakeStates.feeding;
          }
          break;
        case retracting:
          intake.stopFeeder();
          if(coordinator.isClimbing())
          {
            intakeState = IntakeStates.preClimb;
          }
          if (coordinator.canRetract()) {
            intake.retract();
          }
          if (coordinator.getIntakeButtonPressed()
              && (!coordinator.noteInRobot())) {
            intakeState = IntakeStates.deploying;
          } else if (coordinator.isIntakeRetracted()) {
            intakeState = IntakeStates.retracted;
          }
          break;
        case preClimb://on switched to, stop feeding
          if(coordinator.canDeploy())
          {
            intake.deploy();
          }
          else if(coordinator.canRetract())
          {
            intake.retract();
          }
          if(intake.DeployAtTarget(IntakeConstants.DeployConfig.climbingIntakePosition))
          {
            intake.setDeployerBrakeMode();
            intake.stopDeployer();
            intakeState = IntakeStates.Climb;
          }
          break;
        case Climb:
          if(!coordinator.isClimbing())
          {
            intakeState = IntakeStates.retracting;
          }
      }
    }
    else
    {
      //if necessary, we can set position here and wrap line 105 in an isatposition
      intake.stopDeployer();
      intake.stopFeeder();
      
    }
  }
  @Override
  public boolean isFinished() {
    return false;
  }

  public static IntakeStates getIntakeState() {
    return intakeState;
  }

  public static void setIntakeState(IntakeStates newState) {
    intakeState = newState;
  }
}
