package frc.utility;

public class OrangePIDController {
  
  private double kP;

  public OrangePIDController(double kP) {
    this.kP = kP;
  }

  public double calculate(double currentSetpoint, double currentVal, double targetVal) {
    return currentSetpoint + kP * (targetVal - currentVal);
  }

  public void setKp(double newKp) {
    kP = newKp;
  }

  public double getKp() {
    return kP;
  }

}
