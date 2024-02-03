package frc.robot.subsystems.outtakePivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.utility.OrangeMath;

public class OuttakePivotIOReal implements OuttakePivotIO {
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    public OuttakePivotIOReal() {
        pivotMotor = new TalonFX(Constants.OuttakeConstants.pivotDeviceID);
        configPivot();
        configEncoder();
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
    public void updateInputs(OuttakePivotIOInputs inputs) {
        inputs.pivotRotations = pivotMotor.getPosition().getValue();
        inputs.pivotRotationsPerSec = pivotMotor.getVelocity().getValue() / 60;
        inputs.pivotAppliedVolts = pivotMotor.getDutyCycle().getValue() / 2 * pivotMotor.getSupplyVoltage().getValue();
        inputs.pivotCurrentAmps = pivotMotor.getSupplyCurrent().getValue();
        inputs.pivotTempC = pivotMotor.getDeviceTemp().getValue();
        inputs.pivotIsAlive = pivotMotor.isAlive();
    }

    @Override
    public boolean initPivot() {
        pivotMotor.setPosition(pivotEncoder.getAbsolutePosition().getValue());
        return OrangeMath.equalToTwoDecimal(pivotEncoder.getVelocity().getValue(), 0);
    }

    @Override
    public void setPivotTarget(double rotations) {
        pivotMotor.setControl(new PositionVoltage(rotations));
    }

    @Override
    public void setBrakeMode() {
        MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
        mOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().refresh(mOutputConfigs);
        Logger.recordOutput("OuttakePivot/Hardware/NeutralMode", "Brake");
    }

    @Override
    public void setCoastMode() {
        MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
        mOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        pivotMotor.getConfigurator().refresh(mOutputConfigs);
        Logger.recordOutput("OuttakePivot/Hardware/NeutralMode", "Coast");
    }

    @Override
    public void stopPivot() {
        pivotMotor.stopMotor();
    }
    
}
