package frc.robot.shooting;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utility.interpolation.GenericCalculator;
import frc.utility.interpolation.GenericFiringSolutionManager;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

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
  }

  public void writeSolution(FiringSolution solution) {
    try {
      addSolution(solution);
      objectMapper.writeValue(new File("/media/sda1/FiringSolutions.json"), solutions);
      DriverStation.reportWarning("Wrote new solution to firing solution json", false);
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Failed to write new firing solution", null);
    }
  }

  public void loadSolutions() {
    List<FiringSolution> solutionList;
    try {
      solutionList =
          objectMapper.readValue(
              new File("/home/lvuser/deploy/FiringSolutions.json"),
              new TypeReference<List<FiringSolution>>() {});
      for (FiringSolution solution : solutionList) {
        addSolution(solution);
      }
      DriverStation.reportWarning("Loaded all firing solutions", null);
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Failed to load firing solutions", e.getStackTrace());
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