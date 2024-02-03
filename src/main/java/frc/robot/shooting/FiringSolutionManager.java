package frc.robot.shooting;

import java.util.ArrayList;

import frc.utility.interpolation.GenericCalculator;
import frc.utility.interpolation.GenericFiringSolutionManager;

public class FiringSolutionManager implements GenericFiringSolutionManager<FiringSolution> {
  private final ArrayList<FiringSolution> solutions;
  private final GenericCalculator<FiringSolution> calculator;

  public FiringSolutionManager(ArrayList<FiringSolution> solutionArrayList, GenericCalculator<FiringSolution> calculator) {
    solutions = solutionArrayList;
    this.calculator = calculator;
    calculator.init(solutions);
  }

  public void addSolution(FiringSolution solution) {
    solutions.add(solution);
    calculator.whenAdded();
  }

  public FiringSolution calcSolution(double currentMag, double currentDeg) {
    FiringSolution inputsToFind = new FiringSolution(currentMag, currentDeg);
    ArrayList<FiringSolution> selectedSolutions = calculator.find(inputsToFind);
    ArrayList<Double> calculatedComponents =
        calculator.calculate(currentMag, currentDeg, selectedSolutions);
    return new FiringSolution(currentMag, currentDeg, calculatedComponents);
  }
}
