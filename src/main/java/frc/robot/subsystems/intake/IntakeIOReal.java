package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX intake;

  public IntakeIOReal() {
    intake = new TalonFX(IntakeConstants.intakeMotorID);

    configIntake();
  }

  private void configIntake() {
    intake.getConfigurator().apply(new TalonFXConfiguration());

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.NeutralMode = IntakeConstants.IntakeConfig.neutralMode;

    intake.getConfigurator().apply(motorOutputConfigs);

    intake
        .getVelocity()
        .setUpdateFrequency(
            IntakeConstants.IntakeConfig.updateHz, IntakeConstants.IntakeConfig.timeoutMs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRotations = intake.getPosition().getValue();
    inputs.intakeRotationsPerSec = intake.getVelocity().getValue() / 60;
    inputs.intakeAppliedVolts =
        intake.getDutyCycle().getValue() / 2 * intake.getSupplyVoltage().getValue();
    inputs.intakeCurrentAmps = intake.getSupplyCurrent().getValue();
    inputs.intakeTempC = intake.getDeviceTemp().getValue();
    inputs.intakeIsAlive = intake.isAlive();
    inputs.intakeSpeedPct = intake.get();
  }

  @Override
  public void setIntakeRPM(double rpm) {
    intake.set(rpm / IntakeConstants.Intake.maxIntakeRPM);
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intake.getConfigurator().refresh(motorOutputConfigs);

    Logger.recordOutput(IntakeConstants.Logging.hardwareOutputsKey + "NeutralMode", "Brake");
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    intake.getConfigurator().refresh(motorOutputConfigs);

    Logger.recordOutput(IntakeConstants.Logging.hardwareOutputsKey + "NeutralMode", "Coast");
  }

  @Override
  public void stopIntake() {
    intake.stopMotor();
  }
}
