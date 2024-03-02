package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
public class ClimberIOReal implements ClimberIO{
    private final TalonFX climber;
    ShuffleboardTab tab;
    ClimberIOReal()
    {
        climber = new TalonFX(ClimberConstants.climberMotorID);
        configClimber();
        if(Constants.debug)
        {
            tab = Shuffleboard.getTab("Climber");
        }
    }
    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        inputs.rotations = climber.getPosition().getValue();
        inputs.RPS = climber.getVelocity().getValue();
        inputs.appliedVolts = climber.getDutyCycle().getValue() / 2 * climber.getSupplyVoltage().getValue();
        inputs.currentAmps = climber.getSupplyCurrent().getValue();
        inputs.tempC = climber.getDeviceTemp().getValue();
        inputs.isAlive = climber.isAlive();
    }
    private void configClimber()
    {
        climber.getConfigurator().apply(new TalonFXConfiguration());
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    }
    @Override
    public void setClimberVoltage(double voltage)
    {
        climber.setControl(new VoltageOut(voltage));
        Logger.recordOutput("Climber/voltage", voltage);
    }
    @Override
    public void setBrakeMode()
    {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        climber.getConfigurator().refresh(motorOutputConfigs);
    }
    @Override
    public void setCoastMode()
    {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        climber.getConfigurator().refresh(motorOutputConfigs);
    }
}
