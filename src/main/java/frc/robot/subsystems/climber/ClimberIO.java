package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs
    {
        public double rotations = 0.0;
        public double RPS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempC = 0;
        public boolean isAlive = false;
        public double fastVolts = 0.0;
        public double slowVolts = 0.0;
    }
    public default void updateInputs(ClimberIOInputs inputs) {}
    public default void setClimberVoltage(double volts) {}
    public default void setBrakeMode(){}
    public default void setCoastMode(){}
    public default void stopMotor(){}
}
