package frc.robot.shooting;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.utility.interpolation.GenericCalculator;
import frc.utility.interpolation.GenericFiringSolutionManager;
import java.io.File;
import java.io.IOException;
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
      objectMapper.updateValue(new File("/media/sda1/FiringSolutions.json"), solution);
    } catch (JsonMappingException e) {
      e.printStackTrace();
    }
  }

  public void loadSolutions() {
    List<FiringSolution> solutionList;
    try {
      solutionList =
          objectMapper.readValue(
              new File("/media/sda1/FiringSolutions.json"),
              new TypeReference<List<FiringSolution>>() {});
      for (FiringSolution solution : solutionList) {
        addSolution(solution);
      }
    } catch (StreamReadException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (DatabindException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
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
