package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
  @AutoLog
  public class LedIOInputs {
    public double temperatureC = 0.0;
    public double currentAmps = 0.0;
  }

  public default void setLED(int red, int green, int blue, int startOffset, int ledNum) {}
  ;

  public default void rainbowAnimate(
      double brightness, double speed, int ledNum, int startOffset) {}

  public default void flashAnimate(
      int red, int green, int blue, double speed, int ledNum, int startOffset) {}
  ;

  public default void updateInputs(LedIOInputs inputs) {}
  ;
}
