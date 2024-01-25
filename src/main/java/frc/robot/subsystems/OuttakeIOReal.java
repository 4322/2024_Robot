package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.OuttakeConstants;
import frc.utility.OrangeMath;

public class OuttakeIOReal implements OuttakeIO {
    private TalonFX topOuttakeMotor;
    private TalonFX bottomOuttakeMotor;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    public OuttakeIOReal() {
        topOuttakeMotor = new TalonFX(Constants.OuttakeConstants.topOuttakeDeviceID);
        bottomOuttakeMotor = new TalonFX(Constants.OuttakeConstants.bottomOuttakeDeviceID);
        pivotMotor = new TalonFX(Constants.OuttakeConstants.pivotDeviceID);
        configOuttake(topOuttakeMotor);
        configOuttake(bottomOuttakeMotor);
        configPivot();
        configEncoder();

    }

    private void configOuttake(TalonFX talon) {
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.OuttakeConstants.kP;
        slot0Configs.kI = Constants.OuttakeConstants.kI;
        slot0Configs.kD = Constants.OuttakeConstants.kD;
        slot0Configs.kV = Constants.OuttakeConstants.kF;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = Constants.OuttakeConstants.closedLoopRampSec;
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = Constants.OuttakeConstants.openLoopRampSec;
        // bottomOuttakeMotor.setControl(new
        // Follower(topOuttakeMotor.getDeviceID(),true));
        talon.getConfigurator().apply(slot0Configs);
        talon.getConfigurator().apply(closedLoopRampsConfigs);
        talon.getConfigurator().apply(openLoopRampsConfigs);
    }

    private void configPivot() {
        Slot0Configs slot0Configs = new Slot0Configs();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        slot0Configs.kP = Constants.OuttakeConstants.pivotkP;
        slot0Configs.kI = Constants.OuttakeConstants.pivotkI;
        slot0Configs.kD = Constants.OuttakeConstants.pivotkD;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = Constants.OuttakeConstants.pivotClosedLoopSec;
        voltageConfigs.PeakForwardVoltage = Constants.OuttakeConstants.peakPivotVoltage;
        voltageConfigs.PeakReverseVoltage = -Constants.OuttakeConstants.peakPivotVoltage;
        motorOutputConfigs.NeutralMode = Constants.OuttakeConstants.pivotDefaultNeutralMode;
        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotMotor.getConfigurator().apply(closedLoopRampsConfigs);
        pivotMotor.getConfigurator().apply(voltageConfigs);
        pivotMotor.getConfigurator().apply(motorOutputConfigs);
    }

    private void configEncoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        pivotEncoder.getConfigurator().apply(canCoderConfig);
    }

    @Override
    public void updateInputs(OuttakeIOInputs inputs) {
        inputs.topCurrentAmps = topOuttakeMotor.getSupplyCurrent().getValue();
        inputs.topTempC = topOuttakeMotor.getDeviceTemp().getValue();
        inputs.topRotationsPerSec = topOuttakeMotor.getVelocity().getValue();

        inputs.bottomCurrentAmps = bottomOuttakeMotor.getSupplyCurrent().getValue();
        inputs.bottomTempC = bottomOuttakeMotor.getDeviceTemp().getValue();
        inputs.bottomRotationsPerSec = bottomOuttakeMotor.getVelocity().getValue();

        inputs.pivotRotations = pivotMotor.getPosition().getValue();
        inputs.pivotRotationsPerSec = pivotMotor.getVelocity().getValue() / 60;
        inputs.pivotAppliedVolts = pivotMotor.getDutyCycle().getValue() / 2 * pivotMotor.getSupplyVoltage().getValue();
        inputs.pivotCurrentAmps = pivotMotor.getSupplyCurrent().getValue();
        inputs.pivotTempC = pivotMotor.getDeviceTemp().getValue();
        inputs.pivotIsAlive = pivotMotor.isAlive();

        inputs.bottomOuttakeIsAlive = bottomOuttakeMotor.isAlive();
        inputs.topOuttakeIsAlive = topOuttakeMotor.isAlive();
    }

    @Override
    public boolean initPivot() {
        pivotMotor.setPosition(pivotEncoder.getAbsolutePosition().getValue());
        return OrangeMath.equalToTwoDecimal(pivotEncoder.getVelocity().getValue(), 0);
    }

    @Override
    public void setOuttakeRPM(double desiredTopVelocityRPM, double desiredBottomVelocityRPM) {
        double desiredTopVelocityPct = desiredTopVelocityRPM / OuttakeConstants.maxRPM;
        double desiredBottomVelocityPct = desiredBottomVelocityRPM / OuttakeConstants.maxRPM;
        topOuttakeMotor.set(desiredTopVelocityPct);
        bottomOuttakeMotor.set(desiredBottomVelocityPct);
    }

    @Override
    public void setPivotTarget(double rotations) {
        pivotMotor.setControl(new PositionVoltage(rotations));
    }

    @Override
    public void setBrakeMode() {
        MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
        mOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        topOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
        bottomOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
        pivotMotor.getConfigurator().refresh(mOutputConfigs);
        Logger.recordOutput("Outtake/Hardware/NeutralMode", "Brake");
    }

    @Override
    public void setCoastMode() {
        MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
        mOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        topOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
        bottomOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
        pivotMotor.getConfigurator().refresh(mOutputConfigs);
        Logger.recordOutput("Outtake/Hardware/NeutralMode", "Coast");
    }

    @Override
    public void stopOuttake() {
        topOuttakeMotor.stopMotor();
        bottomOuttakeMotor.stopMotor();
    }

    @Override
    public void stopPivot() {
        pivotMotor.stopMotor();
    }
}
