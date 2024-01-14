package frc.utility;

public class OrangeMath {

  public static boolean equalToTwoDecimal(double num1, double num2) {
    double epsilon = 0.01;

    return Math.abs(num1 - num2) < epsilon;
  }

  public static double getCircumference(double diameter) {
    return diameter * Math.PI;
  }

  public static double falconRotationsToMeters(double rotUnits, double wheelCircumferenceMeters,
      double gearRatioMotorToWheel) {
    return (rotUnits * wheelCircumferenceMeters) / gearRatioMotorToWheel;
  }

  public static double feetToMeters(double feet) {
    return feet / 3.28084;
  }

  public static double metersToFeet(double meters) {
    return meters * 3.28084;
  }

  public static double inchesToMeters(double inches) {
    return inches / 39.37;
  }

  public static double metersToInches(double meters) {
    return meters * 39.37;
  }

  public static double pythag(double a, double b) {
    return Math.sqrt(a * a + b * b);  // don't use inefficient Math.pow()
  }

  // Solve for a leg
  public static double inversePythag(double hypotenuse, double leg) {
    return Math.sqrt(hypotenuse * hypotenuse - leg * leg);  // don't use inefficient Math.pow()
  }

  // convert angle to range of +/- 180 degrees
  public static double boundDegrees(double angleDegrees) {
    double x = ((angleDegrees + 180) % 360) - 180;
    if (x < -180) {
      x += 360;
    }
    return x;
  }

  //method that converts from milliseconds to hertz, and hertz to milliseconds as the formula for hertz is hertz = 1000/ms and ms = 1000/hertz= hertz*milliseconds = 1000.
  public static double msAndHzConverter(double time) {
    return 1000 / time;
  }
}
