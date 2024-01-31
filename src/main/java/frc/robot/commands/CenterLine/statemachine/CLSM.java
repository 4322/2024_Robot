package frc.robot.commands.CenterLine.statemachine;

import com.github.oxo42.stateless4j.StateMachine;

import frc.robot.commands.CenterLine.ScoreCenterLine.ScoringStrategy;

public class CLSM {
  
  private final StateMachine<CLSMState, CLSMTrigger> stateMachine;

  public enum CLSMState {
    UpperShootToNote1,
    UpperShootToNote2,
    UpperShootToNote3,
    LowerShootToNote3,
    LowerShootToNote4,
    LowerShootToNote5,
    Note1ToUpperShoot,
    Note2ToUpperShoot,
    Note3ToUpperShoot,
    Note3ToLowerShoot,
    Note4ToLowerShoot,
    Note5ToLowerShoot,
    Note1ToNote2,
    Note2ToNote3,
    Note3ToNote4,
    Note4ToNote5,
    Note5ToNote4,
    Note4ToNote3,
    Note3ToNote2,
    Note2ToNote1,
    Done
  }

  public enum CLSMTrigger {
    CollectedNote,
    DidNotCollectNote,
    FinishedShooting
  }

  public CLSM(ScoringStrategy strategy) {
    stateMachine = new StateMachine<CLSMState, CLSMTrigger>(
      CLSMBuilder.getInitialState(strategy),
      CLSMBuilder.getConfig(strategy)
    );
  }

  public CLSMState getState() {
    return stateMachine.getState();
  }

  public void fire(CLSMTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
