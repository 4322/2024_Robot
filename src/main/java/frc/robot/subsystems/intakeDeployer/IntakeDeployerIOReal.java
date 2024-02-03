package frc.robot.subsystems.intakeDeployer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeDeployerConstants;
import frc.utility.OrangeMath;

public class IntakeDeployerIOReal implements IntakeDeployerIO{
    private final TalonFX deploy;
    private final CANcoder deployEncoder;

    public IntakeDeployerIOReal() {
        deploy = new TalonFX(IntakeDeployerConstants.deployMotorID);
        deployEncoder = new CANcoder(IntakeDeployerConstants.deployEncoderID);

        configDeploy();
        configEncoder();
    }

    private void configDeploy() {
        deploy.getConfigurator().apply(new TalonFXConfiguration());

        Slot0Configs slot0Configs = new Slot0Configs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        slot0Configs.kP = IntakeDeployerConstants.DeployConfig.kP;
        slot0Configs.kD = IntakeDeployerConstants.DeployConfig.kD;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = IntakeDeployerConstants.DeployConfig.configCLosedLoopRamp;
        voltageConfigs.PeakForwardVoltage = IntakeDeployerConstants.DeployConfig.maxVoltage;
        voltageConfigs.PeakReverseVoltage = -IntakeDeployerConstants.DeployConfig.maxVoltage;
        motorOutputConfigs.NeutralMode = IntakeDeployerConstants.DeployConfig.neutralMode;

        deploy.getConfigurator().apply(slot0Configs);
        deploy.getConfigurator().apply(closedLoopRampsConfigs);
        deploy.getConfigurator().apply(voltageConfigs);
        deploy.getConfigurator().apply(motorOutputConfigs);

        // don't need rapid position update
        deploy.getPosition().setUpdateFrequency(IntakeDeployerConstants.DeployConfig.updateHz,
            IntakeDeployerConstants.DeployConfig.timeoutMs);
    }

    private void configEncoder() {
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.AbsoluteSensorRange = IntakeDeployerConstants.EncoderConfig.absSensorRange;
        deployEncoder.getConfigurator().apply(canConfig);
    }

    @Override
    public void updateInputs(IntakeDeployerIOInputs inputs) {
        inputs.deployRotations = deploy.getPosition().getValue();
        inputs.deployRotationsPerSec = deploy.getVelocity().getValue() / 60;
        inputs.deployAppliedVolts = deploy.getDutyCycle().getValue() / 2 * deploy.getSupplyVoltage().getValue();
        inputs.deployCurrentAmps = deploy.getSupplyCurrent().getValue();
        inputs.deployTempC = deploy.getDeviceTemp().getValue();
        inputs.deployIsAlive = deploy.isAlive();

        inputs.deployEncoderRotations = deployEncoder.getPosition().getValue();
        inputs.deployEncoderRotationsPerSec = deployEncoder.getVelocity().getValue() / 60;

        inputs.deployAppliedControl = deploy.getAppliedControl().toString();
    }

    @Override
    public boolean initMotorPos() {
        deploy.setPosition(deployEncoder.getAbsolutePosition().getValue());
        return OrangeMath.equalToTwoDecimal(deployEncoder.getVelocity().getValue(), 0);
    }

    @Override
    public void setDeployTarget(double rotations) {
        deploy.setControl(new PositionVoltage(rotations, IntakeDeployerConstants.Deploy.maxVelRotationsPerSec,
            IntakeDeployerConstants.Deploy.enableFOC, IntakeDeployerConstants.Deploy.FF, IntakeDeployerConstants.Deploy.positionVoltageSlot,
            IntakeDeployerConstants.Deploy.overrideBrakeDuringNeutral, IntakeDeployerConstants.Deploy.limitForwardMotion,
            IntakeDeployerConstants.Deploy.limitReverseMotion));
    }

    @Override
    public void setBrakeMode() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        deploy.getConfigurator().refresh(motorOutputConfigs);

        Logger.recordOutput(IntakeDeployerConstants.Logging.hardwareOutputsKey + "NeutralMode", "Brake");
    }

    @Override
    public void setCoastMode() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        deploy.getConfigurator().refresh(motorOutputConfigs);

        Logger.recordOutput(IntakeDeployerConstants.Logging.hardwareOutputsKey + "NeutralMode", "Coast");
    }

    @Override
    public void stopDeploy() {
        deploy.stopMotor();
    }
}
