package frc.utility.interpolation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class Calculator1D<S extends GenericFiringSolution> implements GenericCalculator<S> {

  private class SortSolution1D<T extends GenericFiringSolution> implements Comparator<T> {
    @Override
    public int compare(T o1, T o2) {
      if (o1.getShotMag() < o2.getShotMag()) {
        return -1;
      } else if (o1.getShotMag() == o2.getShotMag()) {
        return 0;
      } else {
        return 1;
      }
    };
  }

  private ArrayList<S> solutions;

  public void init(ArrayList<S> solutionArrayList) {
    solutions = solutionArrayList;
    solutions.sort(new SortSolution1D<>());
  }

  public ArrayList<S> find(S inputsToFind) {
    ArrayList<S> toReturn = new ArrayList<>();
    int i = Collections.binarySearch(solutions, inputsToFind, new SortSolution1D<>());

    if (i >= 0) {
      toReturn.add(solutions.get(i));
    } else if (i < -solutions.size()) {
      // upper bound
      toReturn.add(solutions.get(solutions.size() - 1));
    } else if (i == -1) {
      // lower bound
      toReturn.add(solutions.get(0));
    } else {
      // convert to insertion point (first greatest element in list)
      // since this is a unique solution, the index below this must be
      // the first lowest element in the list.
      int upperIdx = -(i + 1);
      int lowerIdx = upperIdx - 1;
      toReturn.add(solutions.get(upperIdx));
      toReturn.add(solutions.get(lowerIdx));
    }

    return toReturn;
  }

  public ArrayList<Double> calculate(double currentMag, double currentDeg,
      ArrayList<S> foundSolutions) {

    if (foundSolutions.size() == 1) {
      return foundSolutions.get(0).toComponentList();
    }

    ArrayList<Double> s1ComponentList = foundSolutions.get(0).toComponentList();
    ArrayList<Double> s2ComponentList = foundSolutions.get(1).toComponentList();
    ArrayList<Double> calculatedComponentList = new ArrayList<>();
    // linear interpolation calculation referenced from
    // https://github.com/Team3309/FRC2020/blob/master/src/main/java/frc/robot/util/FiringSolutionManager.java
    double x = (currentMag - foundSolutions.get(0).getShotMag())
        / (foundSolutions.get(1).getShotMag() - foundSolutions.get(0).getShotMag());
    for (int i = 0; i < s1ComponentList.size(); i++) {
      calculatedComponentList.add(s2ComponentList.get(i) * x + s1ComponentList.get(i) * (1 - x));
    }
    return calculatedComponentList;
  }

  public void whenAdded() {
    solutions.sort(new SortSolution1D<>());
  }
}
