package frc.utility.interpolation;

import java.util.ArrayList;

public interface GenericCalculator<S extends GenericFiringSolution> {
  public void init(ArrayList<S> solutionArrayList);

  public ArrayList<S> find(S inputsToFind);

  public ArrayList<Double> calculate(double currentMag, double currentDeg,
      ArrayList<S> foundSolutions);

  public void whenAdded();
}
