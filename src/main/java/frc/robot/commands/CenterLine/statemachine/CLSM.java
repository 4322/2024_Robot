package frc.robot.commands.CenterLine.stateMachine;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import frc.robot.commands.CenterLine.ScoreCenterLine.ScoringStrategy;
import frc.robot.commands.CenterLine.environmentTracker.NoteStatus;

public class CLSM {

  private final StateMachine<CLSMState, CLSMTrigger> stateMachine;
  private TravelState travelState = TravelState.None;
  private NoteStatus noteStatus;

  public enum CLSMState {
    UpperShoot,
    MiddleShoot,
    BottomShoot,
    Note1,
    Note2,
    Note3,
    Note4,
    Note5,
    Done
  }

  public enum CLSMTrigger {
    HaveNote,
    NoNote,
    FinishedShooting,
    FinishedDriving
  }

  public enum TravelState {
    None,
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
    N5ToBS,
    USToN1,
    USToN2,
    USToN3,
    MSToN3,
    MSToN4,
    BSToN4,
    BSToN5,
    Done
  }

  public CLSM(ScoringStrategy strategy) {
    stateMachine = new StateMachine<CLSMState, CLSMTrigger>(
      getInitialState(strategy),
      getConfig(strategy)
    );
    noteStatus = new NoteStatus(true, true, true, true, true);
  }

  private CLSMState getInitialState(ScoringStrategy strategy) {
    switch (strategy) {
      case OneToFive:
        return CLSMState.UpperShoot;
      default: // default to DoNothing
        return CLSMState.Done;
    }
  }

  private StateMachineConfig<CLSMState, CLSMTrigger> getConfig(ScoringStrategy strategy) {
    final StateMachineConfig<CLSMState, CLSMTrigger> config = new StateMachineConfig<>();
    switch (strategy) {
      case OneToFive:
        config.configure(CLSMState.Note1)
          .permit(CLSMTrigger.HaveNote, CLSMState.UpperShoot, () -> setTravelState(TravelState.N1ToUS))
          .permit(CLSMTrigger.NoNote, CLSMState.Note2, () -> setTravelState(TravelState.N1ToN2));
        config.configure(CLSMState.Note2)
          .permit(CLSMTrigger.HaveNote, CLSMState.UpperShoot, () -> setTravelState(TravelState.N2ToUS))
          .permit(CLSMTrigger.NoNote, CLSMState.Note3, () -> setTravelState(TravelState.N2ToN3));
        config.configure(CLSMState.Note3)
          .permit(CLSMTrigger.HaveNote, CLSMState.UpperShoot, () -> setTravelState(TravelState.N3ToMS))
          .permit(CLSMTrigger.NoNote, CLSMState.Note4, () -> setTravelState(TravelState.N3ToN4));
        config.configure(CLSMState.Note4)
          .permit(CLSMTrigger.HaveNote, CLSMState.UpperShoot, () -> setTravelState(TravelState.N4ToBS))
          .permit(CLSMTrigger.NoNote, CLSMState.Note4, () -> setTravelState(TravelState.N4ToN5));
        config.configure(CLSMState.Note5)
          .permit(CLSMTrigger.HaveNote, CLSMState.UpperShoot, () -> setTravelState(TravelState.N5ToBS))
          .permit(CLSMTrigger.NoNote, CLSMState.Note5, () -> setTravelState(TravelState.Done));
        config.configure(CLSMState.UpperShoot)
          .permitIf(CLSMTrigger.FinishedShooting, CLSMState.Note1, () -> noteStatus.note1Available, () -> setTravelState(TravelState.USToN1))
          .permitIf(CLSMTrigger.FinishedShooting, CLSMState.Note2, () -> noteStatus.note2Available, () -> setTravelState(TravelState.USToN2))
          .permitIf(CLSMTrigger.FinishedShooting, CLSMState.Note3, () -> noteStatus.note3Available, () -> setTravelState(TravelState.USToN3));
        config.configure(CLSMState.MiddleShoot)
          .permitIf(CLSMTrigger.FinishedShooting, CLSMState.Note4, () -> noteStatus.note4Available, () -> setTravelState(TravelState.MSToN4));
        config.configure(CLSMState.BottomShoot)
          .permitIf(CLSMTrigger.FinishedShooting, CLSMState.Note5, () -> noteStatus.note5Available, () -> setTravelState(TravelState.BSToN5));
        break;
      default: // empty config
        break;
    }
    return config;
  }

  private void setTravelState(TravelState state) {
    travelState = state;
  }

  public CLSMState getState() {
    return stateMachine.getState();
  }

  public TravelState getTravelState() {
    return travelState;
  }

  public void fire(CLSMTrigger trigger, NoteStatus status) {
    noteStatus = status;
    stateMachine.fire(trigger);
  }
}
