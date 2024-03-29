package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.Constants;

public class LedIOReal implements LedIO {
  public CANdle candle;

  public LedIOReal() {
    candle = new CANdle(Constants.LED.CANdleID, Constants.DriveConstants.Drive.canivoreName);

    CANdleConfiguration config = new CANdleConfiguration();
    config.v5Enabled = true;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1;
    config.vBatOutputMode = VBatOutputMode.On;
    candle.configAllSettings(config);

    // Clear animation stored in CANdle from previous code so that when redeploying,
    // LED configuration will change even if we are in same LED state.
    // We normally only clear animation when we change LED states
    candle.clearAnimation(0);
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.temperatureC = candle.getTemperature();
    inputs.currentAmps = candle.getCurrent();
    inputs.fiveVRailVoltage = candle.get5VRailVoltage();
  }

  @Override
  public void setLED(int red, int green, int blue, int startOffset, int ledNum) {
    candle.setLEDs(red, green, blue, 0, startOffset, ledNum);
  }

  @Override
  public void rainbowAnimate(double brightness, double speed, int startOffset, int ledNum) {
    RainbowAnimation rainbowAnim =
        new RainbowAnimation(brightness, speed, ledNum, false, startOffset);
    candle.animate(rainbowAnim);
  }

  @Override
  public void flashAnimate(
      int red, int green, int blue, double speed, int startOffset, int ledNum) {
    StrobeAnimation strobeAnim =
        new StrobeAnimation(red, green, blue, 0, speed, ledNum, startOffset);
    candle.animate(strobeAnim);
  }

  @Override
  public void fireAnimate(
      double brightness,
      double speed,
      int numLED,
      double sparking,
      double cooling,
      boolean reverse,
      int offset) {
    FireAnimation fireAnimation =
        new FireAnimation(brightness, speed, numLED, sparking, cooling, reverse, offset);
    candle.animate(fireAnimation);
  }

  public void clearAnimation() {
    // clears animation slot so rainbow or strobe won't override setLED
    candle.clearAnimation(0);
  }

  public void configBrightness(double brightness) {
    candle.configBrightnessScalar(brightness);
  }
}
