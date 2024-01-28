package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {
  private Pigeon2 gyro;

  public GyroIOPigeon() {
    gyro = new Pigeon2(0, "rio");
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // log raw rotation values which are bounded from -180 to 180
    inputs.rollPositionDeg =
        gyro.getRoll().getValueAsDouble(); // rotation around x axis (WPI axis convenetion)
    inputs.pitchPositionDeg =
        gyro.getPitch().getValueAsDouble(); // rotation around y axis (WPI axis convenetion)
    inputs.yawPositionDeg =
        -gyro.getYaw().getValueAsDouble(); // rotation around z axis (WPI axis convenetion)

    // log raw accelerometer values
    inputs.accelX = gyro.getAccelerationX().getValueAsDouble();
    inputs.accelY = gyro.getAccelerationY().getValueAsDouble();
    inputs.accelZ = gyro.getAccelerationZ().getValueAsDouble();

    // yaw values used in Drive.java
    inputs.yawAngleDeg = -gyro.getAngle(); // continuous value of yaw position
    inputs.yawVelocityDegPerSec = -gyro.getRate();

    if (gyro.getYaw().hasUpdated()) {
      inputs.connected = true;
    } else {
      inputs.connected = false;
      inputs.yawAngleDeg = 0; // set to robotCentric if gyro disconnects
    }
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
