import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.*;

import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.utility.interpolation.Calculator1D;

public class Calculator1DTest {
  FiringSolutionManager manager;

  @BeforeEach
  public void setup() {
    ArrayList<FiringSolution> solutions = new ArrayList<>();
    solutions.add(new FiringSolution(5, 0, 10, 10));
    solutions.add(new FiringSolution(10, 0, 30, 30));
    solutions.add(new FiringSolution(30, 0, 50, 50));
    manager = new FiringSolutionManager(solutions, new Calculator1D<>());
  }

  @AfterEach
  public void shutdown() {
    return;
  }

  @Test
  public void testCalcNewSolution_ExistingSolution() {
    FiringSolution calcSolution1 = manager.calcSolution(5, 0);
    assertEquals(calcSolution1.getShotMag(), 5);
    assertEquals(calcSolution1.getShotDeg(), 0);
    assertEquals(calcSolution1.getFlywheelSpeed(), 10);
    assertEquals(calcSolution1.getShotAngle(), 10);
  }

  @Test
  public void testCalcNewSolution_NormalSolution() {
    FiringSolution calcSolution1 = manager.calcSolution(7.5, 0);
    assertEquals(calcSolution1.getShotMag(), 7.5);
    assertEquals(calcSolution1.getShotDeg(), 0);
    assertEquals(calcSolution1.getFlywheelSpeed(), 20);
    assertEquals(calcSolution1.getShotAngle(), 20);

    FiringSolution calcSolution2 = manager.calcSolution(15, 0);
    assertEquals(calcSolution2.getShotMag(), 15);
    assertEquals(calcSolution2.getShotDeg(), 0);
    assertEquals(calcSolution2.getFlywheelSpeed(), 35);
    assertEquals(calcSolution2.getShotAngle(), 35);

    FiringSolution calcSolution3 = manager.calcSolution(25, 0);
    assertEquals(calcSolution3.getShotMag(), 25);
    assertEquals(calcSolution3.getShotDeg(), 0);
    assertEquals(calcSolution3.getFlywheelSpeed(), 45);
    assertEquals(calcSolution3.getShotAngle(), 45);
  }

  @Test
  public void testCalcNewSolution_OutsideBounds() {
    FiringSolution calcSolution1 = manager.calcSolution(-5, 0);
    assertEquals(calcSolution1.getShotMag(), -5);
    assertEquals(calcSolution1.getShotDeg(), 0);
    assertEquals(calcSolution1.getFlywheelSpeed(), 10);
    assertEquals(calcSolution1.getShotAngle(), 10);

    FiringSolution calcSolution2 = manager.calcSolution(35, 0);
    assertEquals(calcSolution2.getShotMag(), 35);
    assertEquals(calcSolution2.getShotDeg(), 0);
    assertEquals(calcSolution2.getFlywheelSpeed(), 50);
    assertEquals(calcSolution2.getShotAngle(), 50);
  }

  @Test
  public void testAddNewSolution_NewCalculation() {
    FiringSolution calcSolution1 = manager.calcSolution(15, 0);
    assertEquals(calcSolution1.getShotMag(), 15);
    assertEquals(calcSolution1.getShotDeg(), 0);
    assertEquals(calcSolution1.getFlywheelSpeed(), 35);
    assertEquals(calcSolution1.getShotAngle(), 35);

    manager.addSolution(new FiringSolution(20, 0, 30, 30));

    FiringSolution calcSolution2 = manager.calcSolution(15, 0);
    assertEquals(calcSolution2.getShotMag(), 15);
    assertEquals(calcSolution2.getShotDeg(), 0);
    assertEquals(calcSolution2.getFlywheelSpeed(), 30);
    assertEquals(calcSolution2.getShotAngle(), 30);
  }

  @Test
  public void testAddNewSolution_NewCalculationOutsideBounds() {
    FiringSolution calcSolution1 = manager.calcSolution(-5, 0);
    assertEquals(calcSolution1.getShotMag(), -5);
    assertEquals(calcSolution1.getShotDeg(), 0);
    assertEquals(calcSolution1.getFlywheelSpeed(), 10);
    assertEquals(calcSolution1.getShotAngle(), 10);

    manager.addSolution(new FiringSolution(-1, 0, 30, 30));

    FiringSolution calcSolution2 = manager.calcSolution(-1, 0);
    assertEquals(calcSolution2.getShotMag(), -1);
    assertEquals(calcSolution2.getShotDeg(), 0);
    assertEquals(calcSolution2.getFlywheelSpeed(), 30);
    assertEquals(calcSolution2.getShotAngle(), 30);
  }
}