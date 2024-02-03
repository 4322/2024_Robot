package frc.robot.shooting;

import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.utility.interpolation.GenericCalculator;
import frc.utility.interpolation.GenericFiringSolutionManager;
import java.util.ArrayList;

public class FiringSolutionManager implements GenericFiringSolutionManager<FiringSolution> {
  private final ArrayList<FiringSolution> solutions;
  private final GenericCalculator<FiringSolution> calculator;
  private ObjectMapper objectMapper;

  public FiringSolutionManager(
      ArrayList<FiringSolution> solutionArrayList,
      GenericCalculator<FiringSolution> calculator,
      ObjectMapper objectMapper) {
    solutions = solutionArrayList;
    this.calculator = calculator;
    this.objectMapper = objectMapper;
    calculator.init(solutions);
  }

  public void addSolution(FiringSolution solution) {
    solutions.add(solution);
    calculator.whenAdded();
    try {
      objectMapper.updateValue("/media/sda1/FiringSolutions.JSON", solution);
    } catch (JsonMappingException e) {
      e.printStackTrace();
    }
  }

  public FiringSolution calcSolution(double currentMag, double currentDeg) {
    FiringSolution inputsToFind = new FiringSolution(currentMag, currentDeg);
    ArrayList<FiringSolution> selectedSolutions = calculator.find(inputsToFind);
    ArrayList<Double> calculatedComponents =
        calculator.calculate(currentMag, currentDeg, selectedSolutions);
    return new FiringSolution(currentMag, currentDeg, calculatedComponents);
  }
}
