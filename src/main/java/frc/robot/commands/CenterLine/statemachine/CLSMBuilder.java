package frc.robot.commands.CenterLine.statemachine;

import com.github.oxo42.stateless4j.StateMachineConfig;

import frc.robot.commands.CenterLine.ScoreCenterLine.ScoringStrategy;
import frc.robot.commands.CenterLine.statemachine.CLSM.CLSMState;
import frc.robot.commands.CenterLine.statemachine.CLSM.CLSMTrigger;

public class CLSMBuilder {

  public static CLSMState getInitialState(ScoringStrategy strategy) {
    switch (strategy) {
      case TopThree:
        return CLSMState.UpperShootToNote1;
      default: // default to DoNothing
        return CLSMState.Done;
    }
  }

  public static StateMachineConfig<CLSMState, CLSMTrigger> getConfig(ScoringStrategy strategy) {

    final StateMachineConfig<CLSMState, CLSMTrigger> config = new StateMachineConfig<>();

    switch (strategy) {
      case TopThree:
        config.configure(CLSMState.UpperShootToNote1)
          .permit(CLSMTrigger.CollectedNote, CLSMState.Note1ToUpperShoot)
          .permit(CLSMTrigger.DidNotCollectNote, CLSMState.Note1ToNote2);
        config.configure(CLSMState.Note1ToUpperShoot)
          .permit(CLSMTrigger.FinishedShooting, CLSMState.UpperShootToNote2);
        config.configure(CLSMState.UpperShootToNote2)
          .permit(CLSMTrigger.CollectedNote, CLSMState.Note2ToUpperShoot)
          .permit(CLSMTrigger.DidNotCollectNote, CLSMState.Note2ToNote3);
        config.configure(CLSMState.Note2ToUpperShoot)
          .permit(CLSMTrigger.FinishedShooting, CLSMState.UpperShootToNote3);
        config.configure(CLSMState.UpperShootToNote3)
          .permit(CLSMTrigger.CollectedNote, CLSMState.Note3ToUpperShoot)
          .permit(CLSMTrigger.DidNotCollectNote, CLSMState.Done);
        config.configure(CLSMState.Note3ToUpperShoot)
          .permit(CLSMTrigger.FinishedShooting, CLSMState.Done);
        break;
      default: // empty config
        break;
    }

    return config;
  }

}
