package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.util.Color;

public interface LedIO {
  @AutoLog
  public class LedIOInputs {
    public double temperatureC = 0.0;
    public double currentAmps = 0.0;
    public double fiveVRailVoltage = 0.0;
  }

  public default void setLED(Color color, int startOffset, int ledNum) {}
  ;

  public default void rainbowAnimate(
      double brightness, double speed, int startOffset, int ledNum) {}

  public default void flashAnimate(
      int red, int green, int blue, double speed, int ledNum, int startOffset) {}
  ;

  public default void fireAnimate(
      double brightness,
      double speed,
      int numLED,
      double sparking,
      double cooling,
      boolean reverse,
      int offset) {}

  public default void clearAnimation() {}
  ;

  public default void configBrightness(double brightness) {}
  ;

  public default void updateInputs(LedIOInputs inputs) {}
  ;
}
