package CLSM;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.commands.CenterLine.ScoreCenterLine.ScoringStrategy;
import frc.robot.commands.CenterLine.stateMachine.CLSM;
import frc.robot.commands.CenterLine.stateMachine.CLSM.CLSMState;
import frc.robot.commands.CenterLine.stateMachine.CLSM.TravelState;
import org.junit.jupiter.api.*;

public class OneToFiveTest {
  CLSM machine;

  @BeforeEach
  public void setup() {
    machine = new CLSM(ScoringStrategy.OneToFive);
  }

  @AfterEach
  public void shutdown() {
    return;
  }

  @Test
  public void testInitialStates_UpperShoot() {
    assertEquals(machine.getState(), CLSMState.UpperShoot);
    assertEquals(machine.getTravelState(), TravelState.None);
  }
}
