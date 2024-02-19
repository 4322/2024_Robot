package frc.robot.shooting;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.utility.interpolation.Calculator1D;
import frc.utility.interpolation.GenericCalculator;
import frc.utility.interpolation.GenericFiringSolutionManager;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class FiringSolutionManager implements GenericFiringSolutionManager<FiringSolution> {
  private static ArrayList<FiringSolution> solutions = new ArrayList<FiringSolution>();
  private static GenericCalculator<FiringSolution> calculator = new Calculator1D<>();
  private static ObjectMapper objectMapper = new ObjectMapper();

  private static FiringSolutionManager firingSolutionManager;

  public static final FiringSolutionManager getInstance() {
    if (firingSolutionManager == null) {
      firingSolutionManager = new FiringSolutionManager();
    }
    return firingSolutionManager;
  }

  private FiringSolutionManager() {
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
              new File(Filesystem.getDeployDirectory().getPath() + "/FiringSolutions.json"),
              new TypeReference<ArrayList<FiringSolution>>() {});
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
