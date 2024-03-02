package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants.DeployConfig;
public class ClimberIOReal implements ClimberIO{
    private final TalonFX climber;
    ShuffleboardTab tab;
    GenericEntry climberRotations;
    GenericEntry slowVolts;
    GenericEntry fastVolts;
    ClimberIOReal()
    {
        climber = new TalonFX(ClimberConstants.climberMotorID);
        configClimber();
        if(Constants.debug)
        {
            tab = Shuffleboard.getTab("Climber");
            climberRotations = tab.add("Climber Rotations (read only)", 0)
                .withSize(2,1)
                .withPosition(0, 0).getEntry();
            slowVolts = tab.add("Slow Mode Volts", ClimberConstants.slowClimberVolts)
                .withSize(2, 1)
                .withPosition(2, 0).getEntry();
            fastVolts = tab.add("Fast Mode Volts", ClimberConstants.fastClimberVolts)
                .withSize(2,1)
                .withPosition(4,0).getEntry();

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
        if(Constants.debug)
        {
            inputs.fastVolts = fastVolts.getDouble(ClimberConstants.fastClimberVolts);
            inputs.slowVolts = slowVolts.getDouble(ClimberConstants.slowClimberVolts);
            climberRotations.setDouble(inputs.rotations);
        }
        else
        {
            inputs.fastVolts = ClimberConstants.fastClimberVolts;
            inputs.slowVolts = ClimberConstants.slowClimberVolts;
        }
    }
    private void configClimber()
    {
        climber.getConfigurator().apply(new TalonFXConfiguration());
        Slot0Configs slot0Configs = new Slot0Configs();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = ClimberConstants.openRampPeriod;
        slot0Configs.kP = ClimberConstants.kP;
        slot0Configs.kI = ClimberConstants.kI;
        slot0Configs.kD = ClimberConstants.kD;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        //TODO: Set peakForwardVoltage
        voltageConfigs.PeakForwardVoltage = ClimberConstants.peakForwardVoltage;
        //TODO: Set peakReverseVoltage
        voltageConfigs.PeakReverseVoltage = ClimberConstants.peakReverseVoltage;
        softwareLimitSwitchConfigs.ForwardSoftLimitEnable = ClimberConstants.limitRotations;
        softwareLimitSwitchConfigs.ForwardSoftLimitThreshold =
            ClimberConstants.climberMaxRotations; // TODO: Set constant
        softwareLimitSwitchConfigs.ReverseSoftLimitThreshold =
            ClimberConstants.climberMinRotations; // TODO: Set constant 
        currentLimitsConfigs.StatorCurrentLimitEnable = DeployConfig.statorEnabled;
        currentLimitsConfigs.StatorCurrentLimit = DeployConfig.statorLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = DeployConfig.supplyEnabled;
        currentLimitsConfigs.SupplyCurrentLimit = DeployConfig.supplyLimit;
        climber.getConfigurator().apply(currentLimitsConfigs);
        climber.getConfigurator().apply(softwareLimitSwitchConfigs);
        climber.getConfigurator().apply(voltageConfigs);
        climber.getConfigurator().apply(motorOutputConfigs);
        climber.getConfigurator().apply(slot0Configs);
        climber.getConfigurator().apply(openLoopRampsConfigs);
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
    @Override
    public void stopMotor()
    {
        climber.stopMotor();
    }
}
