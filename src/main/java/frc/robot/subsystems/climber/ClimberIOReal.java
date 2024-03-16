package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    climber.getConfigurator().apply(new TalonFXConfiguration());
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    VoltageConfigs voltageConfigs = new VoltageConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = ClimberConstants.openRampPeriod;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    voltageConfigs.PeakForwardVoltage = ClimberConstants.peakForwardVoltage;
    voltageConfigs.PeakReverseVoltage = ClimberConstants.peakReverseVoltage;
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold =
        ClimberConstants.climberMaxRotations;
    currentLimitsConfigs.StatorCurrentLimitEnable = ClimberConstants.statorEnabled;
    currentLimitsConfigs.StatorCurrentLimit = ClimberConstants.statorLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = ClimberConstants.supplyEnabled;
    currentLimitsConfigs.SupplyCurrentLimit = ClimberConstants.supplyLimit;
    climber.getConfigurator().apply(currentLimitsConfigs);
    climber.getConfigurator().apply(softwareLimitSwitchConfigs);
    climber.getConfigurator().apply(voltageConfigs);
    climber.getConfigurator().apply(motorOutputConfigs);
    climber.getConfigurator().apply(openLoopRampsConfigs);
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
