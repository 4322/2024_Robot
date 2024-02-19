package frc.utility.interpolation;

import java.util.ArrayList;

public abstract class GenericFiringSolution {
  protected abstract ArrayList<Double> toComponentList();

  public abstract double getShotMag();

  public abstract double getShotDeg();
}
