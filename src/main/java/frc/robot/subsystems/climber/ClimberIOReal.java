package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import org.littletonrobotics.junction.Logger;

public class ClimberIOReal implements ClimberIO {
  private final TalonFX climber;

  ClimberIOReal() {
    climber =
        new TalonFX(ClimberConstants.climberMotorID, Constants.DriveConstants.Drive.canivoreName);
    configClimber();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.rotations = climber.getPosition().getValue();
    inputs.RPS = climber.getVelocity().getValue();
    inputs.appliedVolts =
        climber.getDutyCycle().getValue() / 2 * climber.getSupplyVoltage().getValue();
    inputs.currentAmps = climber.getSupplyCurrent().getValue();
    inputs.tempC = climber.getDeviceTemp().getValue();
    inputs.isAlive = climber.isAlive();
  }

  private void configClimber() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = ClimberConstants.openRampPeriod;
    config.Voltage.PeakForwardVoltage = ClimberConstants.peakForwardVoltage;
    config.Voltage.PeakReverseVoltage = ClimberConstants.peakReverseVoltage;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.climberMaxRotations;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ClimberConstants.statorLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.supplyLimit;

    climber.getConfigurator().apply(config);
  }

  /*
   * Used when extending and performing slow override
   * Deson't use FOC because it doesn't lift robot or come in contact with chain
   */
  @Override
  public void setFreeMoveVoltage(double voltage) {
    climber.setControl(new VoltageOut(voltage));
    Logger.recordOutput("Climber/voltage", voltage);
  }

  /*
   * Used ONLY when climbing
   * Needs FOC control due to motor lifting entire weight of robot
   */
  @Override
  public void setClimbingVoltage(double voltage) {
    climber.setControl(new VoltageOut(voltage).withEnableFOC(true));
    Logger.recordOutput("Climber/voltage", voltage);
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    climber.getConfigurator().refresh(motorOutputConfigs);
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    climber.getConfigurator().refresh(motorOutputConfigs);
  }

  @Override
  public void stopMotor() {
    climber.stopMotor();
  }

  @Override
  public void zeroClimberAtCurrentPos() {
    climber.setPosition(0);
  }
}
