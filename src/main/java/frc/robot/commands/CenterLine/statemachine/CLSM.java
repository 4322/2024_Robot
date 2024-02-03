package frc.robot.commands.CenterLine.stateMachine;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import frc.robot.commands.CenterLine.ScoreCenterLine.ScoringStrategy;

public class CLSM {

  private final StateMachine<CLSMState, CLSMTrigger> stateMachine;
  private TravelState currentTravelState;

  public enum CLSMState {
    UpperShoot,
    MiddleShoot,
    BottomShoot,
    Note1,
    Note2,
    Note3,
    Note4,
    Note5,
    BottomEndPoint,
    Done,
  }

  public enum CLSMTrigger {
    CollectedNote,
    DidNotCollectNote,
    FinishedShooting,
    FinishedDriving
  }

  public enum TravelState {
    N1ToN2,
    N2ToN3,
    N3ToN4,
    N4ToN5,
    N5ToN4,
    N4ToN3,
    N3ToN2,
    N2ToN1,
    SnatchN1ToN2,
    SnatchN2ToN3,
    SnatchN3ToN4,
    SnatchN4ToN5,
    N1ToUS,
    N2ToUS,
    N3ToUS,
    N3ToMS,
    N4ToMS,
    N4ToBS,
    N5ToBS
  }

  public CLSM(ScoringStrategy strategy) {
    stateMachine = new StateMachine<CLSMState, CLSMTrigger>(
        getInitialState(strategy),
        getConfig(strategy));
  }

  public CLSMState getInitialState(ScoringStrategy strategy) {
    switch (strategy) {
      case TopThree:
        return CLSMState.UpperShoot;
      default: // default to DoNothing
        return CLSMState.Done;
    }
  }

  public StateMachineConfig<CLSMState, CLSMTrigger> getConfig(ScoringStrategy strategy) {
    final StateMachineConfig<CLSMState, CLSMTrigger> config = new StateMachineConfig<>();

    switch (strategy) {
      case TopThree:
        break;
      default: // empty config
        break;
    }

    return config;
  }

  public CLSMState getState() {
    return stateMachine.getState();
  }



  public void fire(CLSMTrigger trigger) {
    stateMachine.fire(trigger);
  }
}
