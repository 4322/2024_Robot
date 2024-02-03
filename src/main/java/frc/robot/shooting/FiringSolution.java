package frc.robot.shooting;

import frc.utility.interpolation.GenericFiringSolution;
import java.util.ArrayList;

public class FiringSolution extends GenericFiringSolution {
  private double shotMag;
  private double shotDeg; // angle from head-on to target (top-down to field)
  private double flywheelSpeed;
  private double shotAngle; // angle at which we should shoot (robot side view)

  public FiringSolution(double shotMag, double shotDeg, double flywheelSpeed, double shotAngle) {
    this.shotMag = shotMag;
    this.shotDeg = shotDeg;
    this.flywheelSpeed = flywheelSpeed;
    this.shotAngle = shotAngle;
  }

  protected FiringSolution(double shotMag, double shotDeg, ArrayList<Double> componentList) {
    this.shotMag = shotMag;
    this.shotDeg = shotDeg;
    this.flywheelSpeed = componentList.get(0);
    this.shotAngle = componentList.get(0);
  }

  protected FiringSolution(double shotMag, double shotDeg) {
    this.shotMag = shotMag;
    this.shotDeg = shotDeg;
    this.flywheelSpeed = 0;
    this.shotAngle = 0;
  }

  @Override
  protected ArrayList<Double> toComponentList() {
    ArrayList<Double> list = new ArrayList<>();
    list.add(flywheelSpeed);
    list.add(shotAngle);
    return list;
  }

  @Override
  public double getShotMag() {
    return shotMag;
  }

  @Override
  public double getShotDeg() {
    return shotDeg;
  }

  public double getFlywheelSpeed() {
    return flywheelSpeed;
  }

  public double getShotAngle() {
    return shotAngle;
  }
}
