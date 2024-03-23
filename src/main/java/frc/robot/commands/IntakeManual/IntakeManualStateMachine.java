package frc.robot.commands.IntakeManual;

import com.github.oxo42.stateless4j.*;

public class IntakeManualStateMachine {

  private StateMachine<IntakeManualState, IntakeManualTrigger> stateMachine;
  private StateMachineConfig<IntakeManualState, IntakeManualTrigger> config =
      new StateMachineConfig<>();

  public enum IntakeManualState {
    RETRACTED,
    DEPLOYING,
    FEEDING,
    NOTE_OBTAINED,
    NOTE_PAST_INTAKE,
    RETRACTING;
  }

  public enum IntakeManualTrigger {
    ENABLE_RETRACTED,
    ENABLE_DEPLOYING,
    ENABLE_FEEDING,
    ENABLE_NOTEOBTAINED,
    ENABLE_NOTEPASTINTAKE,
    ENABLE_RETRACTING;
  }

  public IntakeManualStateMachine(IntakeManualState initialState) {
    config
        .configure(IntakeManualState.RETRACTED)
        .permitReentry(IntakeManualTrigger.ENABLE_RETRACTED)
        .permit(IntakeManualTrigger.ENABLE_DEPLOYING, IntakeManualState.DEPLOYING)
        .permit(IntakeManualTrigger.ENABLE_FEEDING, IntakeManualState.FEEDING)
        .permit(IntakeManualTrigger.ENABLE_NOTEOBTAINED, IntakeManualState.NOTE_OBTAINED)
        .permit(IntakeManualTrigger.ENABLE_NOTEPASTINTAKE, IntakeManualState.NOTE_PAST_INTAKE)
        .permit(IntakeManualTrigger.ENABLE_RETRACTING, IntakeManualState.RETRACTING);


    config
        .configure(IntakeManualState.DEPLOYING)
        .permit(IntakeManualTrigger.ENABLE_RETRACTED, IntakeManualState.RETRACTED)
        .permitReentry(IntakeManualTrigger.ENABLE_DEPLOYING)
        .permit(IntakeManualTrigger.ENABLE_FEEDING, IntakeManualState.FEEDING)
        .permit(IntakeManualTrigger.ENABLE_NOTEOBTAINED, IntakeManualState.NOTE_OBTAINED)
        .permit(IntakeManualTrigger.ENABLE_NOTEPASTINTAKE, IntakeManualState.NOTE_PAST_INTAKE)
        .permit(IntakeManualTrigger.ENABLE_RETRACTING, IntakeManualState.RETRACTING);

    config
        .configure(IntakeManualState.FEEDING)
        .permit(IntakeManualTrigger.ENABLE_RETRACTED, IntakeManualState.RETRACTED)
        .permit(IntakeManualTrigger.ENABLE_DEPLOYING, IntakeManualState.DEPLOYING)
        .permitReentry(IntakeManualTrigger.ENABLE_FEEDING)
        .permit(IntakeManualTrigger.ENABLE_NOTEOBTAINED, IntakeManualState.NOTE_OBTAINED)
        .permit(IntakeManualTrigger.ENABLE_NOTEPASTINTAKE, IntakeManualState.NOTE_PAST_INTAKE)
        .permit(IntakeManualTrigger.ENABLE_RETRACTING, IntakeManualState.RETRACTING);

    config
        .configure(IntakeManualState.NOTE_OBTAINED)
        .permit(IntakeManualTrigger.ENABLE_RETRACTED, IntakeManualState.RETRACTED)
        .permit(IntakeManualTrigger.ENABLE_DEPLOYING, IntakeManualState.DEPLOYING)
        .permit(IntakeManualTrigger.ENABLE_FEEDING, IntakeManualState.FEEDING)
        .permitReentry(IntakeManualTrigger.ENABLE_NOTEOBTAINED)
        .permit(IntakeManualTrigger.ENABLE_NOTEPASTINTAKE, IntakeManualState.NOTE_PAST_INTAKE)
        .permit(IntakeManualTrigger.ENABLE_RETRACTING, IntakeManualState.RETRACTING);
    
    config
        .configure(IntakeManualState.NOTE_PAST_INTAKE)
        .permit(IntakeManualTrigger.ENABLE_RETRACTED, IntakeManualState.RETRACTED)
        .permit(IntakeManualTrigger.ENABLE_DEPLOYING, IntakeManualState.DEPLOYING)
        .permit(IntakeManualTrigger.ENABLE_FEEDING, IntakeManualState.FEEDING)
        .permit(IntakeManualTrigger.ENABLE_NOTEOBTAINED, IntakeManualState.NOTE_OBTAINED)
        .permitReentry(IntakeManualTrigger.ENABLE_NOTEPASTINTAKE)
        .permit(IntakeManualTrigger.ENABLE_RETRACTING, IntakeManualState.RETRACTING);

    config
        .configure(IntakeManualState.RETRACTING)
        .permit(IntakeManualTrigger.ENABLE_RETRACTED, IntakeManualState.RETRACTED)
        .permit(IntakeManualTrigger.ENABLE_DEPLOYING, IntakeManualState.DEPLOYING)
        .permit(IntakeManualTrigger.ENABLE_FEEDING, IntakeManualState.FEEDING)
        .permit(IntakeManualTrigger.ENABLE_NOTEOBTAINED, IntakeManualState.NOTE_OBTAINED)
        .permit(IntakeManualTrigger.ENABLE_NOTEPASTINTAKE, IntakeManualState.NOTE_PAST_INTAKE)
        .permitReentry(IntakeManualTrigger.ENABLE_RETRACTING);

    stateMachine = new StateMachine<IntakeManualState, IntakeManualTrigger>(initialState, config);
  }

  public IntakeManualState getState() {
    return stateMachine.getState();
  }

  public void fire(IntakeManualTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
