package CLSM;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.commands.CenterLine.ScoreCenterLine;
import frc.robot.commands.CenterLine.ScoreCenterLine.ScoringStrategy;
import frc.robot.commands.CenterLine.statemachine.CLSM.CLSMState;
import frc.robot.commands.CenterLine.statemachine.CLSM.TravelState;
import frc.robot.subsystems.TestRobotCoordinator;
import frc.robot.subsystems.drive.TestDrive;
import org.junit.jupiter.api.*;

public class ScoreCenterLineTest {

  @BeforeEach
  public void setup() {
    return;
  }

  @AfterEach
  public void shutdown() {
    return;
  }

  @Test
  public void testOneToFive_AllCollected() {
    ScoreCenterLine command =
        new ScoreCenterLine(
            new TestDrive(),
            new TestRobotCoordinator(true, true, true, true, true, true),
            ScoringStrategy.OneToFive);

    CLSMState[] stateList = {
      CLSMState.TopShoot,
      CLSMState.Note1,
      CLSMState.TopShoot,
      CLSMState.Note2,
      CLSMState.TopShoot,
      CLSMState.Note3,
      CLSMState.MiddleShoot,
      CLSMState.Note4,
      CLSMState.BottomShoot,
      CLSMState.Note5,
      CLSMState.BottomShoot,
      CLSMState.EndPos,
      CLSMState.Done
    };

    TravelState[] travelStateList = {
      TravelState.None,
      TravelState.USToN1,
      TravelState.N1ToUS,
      TravelState.USToN2,
      TravelState.N2ToUS,
      TravelState.USToN3,
      TravelState.N3ToMS,
      TravelState.MSToN4,
      TravelState.N4ToBS,
      TravelState.BSToN5,
      TravelState.N5ToBS,
      TravelState.BSToEndPos,
      TravelState.Done
    };

    testCommand(command, stateList, travelStateList);
  }

  @Test
  public void testOneToFive_NoneCollected() {
    ScoreCenterLine command =
        new ScoreCenterLine(
            new TestDrive(),
            new TestRobotCoordinator(true, true, true, true, false, false),
            ScoringStrategy.OneToFive);

    CLSMState[] stateList = {
      CLSMState.TopShoot,
      CLSMState.Note1,
      CLSMState.Note2,
      CLSMState.Note3,
      CLSMState.Note4,
      CLSMState.Note5,
      CLSMState.EndPos,
      CLSMState.Done
    };

    TravelState[] travelStateList = {
      TravelState.None,
      TravelState.USToN1,
      TravelState.N1ToN2,
      TravelState.N2ToN3,
      TravelState.N3ToN4,
      TravelState.N4ToN5,
      TravelState.N5ToEndPos,
      TravelState.Done
    };

    testCommand(command, stateList, travelStateList);
  }

  public void testCommand(
      ScoreCenterLine command, CLSMState[] stateList, TravelState[] travelStateList) {
    assertEquals(stateList.length, travelStateList.length);
    for (int i = 0; i < stateList.length; i++) {
      System.out.println(command.getState().toString());
      System.out.println(command.getTravelState().toString());
      System.out.println(command.getNoteStatus().toString());
      assertEquals(command.getState(), stateList[i]);
      assertEquals(command.getTravelState(), travelStateList[i]);
      if (i == 0) {
        command.initialize();
      } else if (!command.isFinished()) {
        command.execute();
      }
    }
  }
}
