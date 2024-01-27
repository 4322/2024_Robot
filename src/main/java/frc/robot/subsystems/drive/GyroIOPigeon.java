package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;

public class GyroIOPigeon implements GyroIO{
    private Pigeon2 gyro;

    public GyroIOPigeon() {
        gyro = new Pigeon2(0, null);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // log raw values which are bounded from -180 to 180
        inputs.rollPositionDeg = gyro.getRoll().getValueAsDouble(); // rotation around x axis (WPI axis convenetion)
        inputs.pitchPositionDeg = gyro.getPitch().getValueAsDouble(); // rotation around y axis (WPI axis convenetion)
        inputs.yawPositionDeg = -gyro.getYaw().getValueAsDouble(); // rotation around z axis (WPI axis convenetion)

        // yaw values used in Drive.java
        inputs.yawAngleDeg = -gyro.getAngle(); // continuous value of yaw position
        inputs.yawVelocityDegPerSec = -gyro.getRate();
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
