package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public double driveRotations = 0.0;
        public double driveRotationsPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double turnVelocityDegPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnDegrees = 0.0;

        public double calculatedFF = 0.0;
    }
    
    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDriveVoltage(double desiredVelocity) {}

    public default void updateFeedForward(double[] newFeedForwardVolts) {}

    public default void setFeedForwardSpeedThreshold(double[] newFeedForwardRPSThreshold) {}

    public default void updateVoltsToOvercomeFriction(double newkSVolts) {}

    public default void setTurnAngle(double desiredAngle) {} // angle from 0 to 360 degrees

    public default void setBrakeMode() {}

    public default void setCoastMode() {}

    public default void setClosedRampRate(double period) {}

    public default void setOpenRampRate(double period) {}

    public default void stopMotor() {}

}
