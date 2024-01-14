package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.utility.CanBusUtil;
import frc.utility.OrangeMath;

public class SwerveModuleIOMotorControl implements SwerveModuleIO {
    private TalonFX driveMotor;

    private TalonFX turningMotor;
    private CANcoder encoder;
    
    private double[] feedForwardRPSThreshold = DriveConstants.Drive.FeedForward.feedForwardRPSThreshold.clone();
    private double[] feedForwardVolts = DriveConstants.Drive.FeedForward.voltsAtSpeedThresholds.clone();
    private double kSVolts = DriveConstants.Drive.kS;

    private double calculatedFeedForwardValue;

    public SwerveModuleIOMotorControl(WheelPosition wheelPos) {
        switch(wheelPos) {
            case FRONT_RIGHT:
                driveMotor = new TalonFX(DriveConstants.frontRightDriveID);
                turningMotor = new TalonFX(DriveConstants.frontRightRotationID);
                encoder = new CANcoder(DriveConstants.frontRightCANID);
                break;
            case FRONT_LEFT:
                driveMotor = new TalonFX(DriveConstants.frontLeftDriveID);
                turningMotor = new TalonFX(DriveConstants.frontLeftRotationID);
                encoder = new CANcoder(DriveConstants.frontLeftCANID);
                break;
            case BACK_RIGHT:
                driveMotor = new TalonFX(DriveConstants.rearRightDriveID);
                turningMotor = new TalonFX(DriveConstants.rearRightRotationID);
                encoder = new CANcoder(DriveConstants.rearRightCANID);
                break;
            case BACK_LEFT: 
                driveMotor = new TalonFX(DriveConstants.rearLeftDriveID);
                turningMotor = new TalonFX(DriveConstants.rearLeftRotationID);
                encoder = new CANcoder(DriveConstants.rearLeftCANID);
                break;
        }
        
        configDrive(driveMotor, wheelPos);

        configRotation(turningMotor);
    }

    private void configDrive(TalonFX talonFX, WheelPosition pos) {
        talonFX.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfigurator config = talonFX.getConfigurator();
        Slot0Configs slot0Configs = new Slot0Configs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        slot0Configs.kP = DriveConstants.Drive.kP;
        slot0Configs.kI = DriveConstants.Drive.kI;
        slot0Configs.kD = DriveConstants.Drive.kD;
        slot0Configs.kV = DriveConstants.Drive.kF;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = DriveConstants.Drive.closedLoopRampSec;
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = DriveConstants.Drive.openLoopRampSec;
        currentLimitsConfigs.StatorCurrentLimit = DriveConstants.Drive.statorLimit;
        currentLimitsConfigs.SupplyCurrentLimit = DriveConstants.Drive.supplyLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = DriveConstants.Drive.supplyEnabled;
        currentLimitsConfigs.SupplyCurrentThreshold = DriveConstants.Drive.supplyThreshold;
        //TODO: sparkMax.enableVoltageCompensation(DriveConstants.Drive.voltageCompSaturation);
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        config.apply(slot0Configs);
        config.apply(closedLoopRampsConfigs);
        config.apply(openLoopRampsConfigs);
        config.apply(currentLimitsConfigs);
        config.apply(motorOutputConfigs);
        // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
        // Without this code, all bevel gears would need to face right when the modules are zeroed.
        boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
        talonFX.setInverted(isLeftSide);

        // need rapid velocity feedback for control logic
        talonFX.getVelocity().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), Constants.controllerConfigTimeoutMs);
      }

      private void configRotation(TalonFX talonFX) {
        talonFX.getConfigurator().apply(new TalonFXConfiguration());

        Slot0Configs slot0Configs = new Slot0Configs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
  
        talonFX.setInverted(true);
      
        TalonFXConfigurator config = talonFX.getConfigurator();
        slot0Configs.kP = DriveConstants.Rotation.kP;
        slot0Configs.kD = DriveConstants.Rotation.kD;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = DriveConstants.Rotation.configCLosedLoopRamp;
        closedLoopGeneralConfigs.ContinuousWrap = true;
        //TODO: config.setSmartMotionAllowedClosedLoopError(DriveConstants.Rotation.allowableClosedloopError,0);
        voltageConfigs.PeakForwardVoltage = DriveConstants.Rotation.maxPower;
        voltageConfigs.PeakReverseVoltage = -DriveConstants.Rotation.maxPower;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;// Allow robot to be moved prior to enabling
        //TODO: sparkMax.enableVoltageCompensation(DriveConstants.Rotation.configVoltageCompSaturation); 
        //TODO: set stator and supply limits for rotation moto
        //encoder.setPositionConversionFactor(360);  TODO: rewrite code to operate on 0-1 rotations instead of 0-360 degrees. convert encoder position duty cycle to degrees
        CANSparkMax canSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        canSparkMax.setSmartCurrentLimit(0, 0);
        //config.setFeedbackDevice(encoder); 

        canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder.getConfigurator().apply(canConfig);
        talonFX.getConfigurator().apply(slot0Configs);
        talonFX.getConfigurator().apply(closedLoopRampsConfigs);
        talonFX.getConfigurator().apply(closedLoopGeneralConfigs);
        talonFX.getConfigurator().apply(voltageConfigs);
        talonFX.getConfigurator().apply(motorOutputConfigs);
        // need rapid position feedback for steering control
        talon.getPosition().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
      Constants.controllerConfigTimeoutMs);
      }

      // Below are the implementations of the methods in SwerveModuleIO.java
      @Override
      public void updateInputs(SwerveModuleIOInputs inputs) {
        //drive inputs
        inputs.driveRotations = driveMotor.getPosition().getValue();
        inputs.driveRotationsPerSec = driveMotor.getVelocity().getValue()/60;
        inputs.driveAppliedVolts = driveMotor.getDutyCycle().getValue()/2 * driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps = driveMotor.getSupplyCurrent().getValue();
        //turn inputs
        inputs.turnVelocityDegPerSec = Units.rotationsToDegrees(turningMotor.getVelocity().getValue());
        inputs.turnAppliedVolts = turningMotor.getDutyCycle().getValue()/2 * turningMotor.getSupplyVoltage().getValue();
        inputs.turnCurrentAmps = turningMotor.getSupplyCurrent().getValue();
        inputs.turnDegrees = turningMotor.getPosition().getValue();

        inputs.calculatedFF = calculatedFeedForwardValue;
    }

    // PID methods for turn motor
    @Override
    public void setTurnAngle(double desiredAngle) {
           PositionVoltage positionVoltage = new PositionVoltage(desiredAngle);
      turningMotor.setControl(positionVoltage);
    }

    // PID method for drive motor
    @Override
    public void setDriveVoltage(double desiredMotorRPM) {
      // convert RPM to RPS
      double desiredMotorRPS = Math.abs(desiredMotorRPM / 60);
      int i;
      
      // If motor RPS less than element 1 of FF speed threshold, i defaults to 0
      // due to for loop iterating i at the end.
      // Greater than or equals to not used in order to protect against erroneous shuffleboard input
      // where element 0 of FF velocity threshold is changed from 0.
      for (i = feedForwardRPSThreshold.length - 1; i > 0; i--) { 
        if (desiredMotorRPS >= feedForwardRPSThreshold[i]) {
          break;
        }
      }

      // Linear extrapolation to account for edge case where requested speed 
      // is greater than max threshold speed.
      // Feed Forward calculated by determining linear equation between last two points.
      if (i == feedForwardRPSThreshold.length - 1) {
        int lastElement = i;
        int secondToLastElement = i - 1;
        // Slope between last point and second to last point used to predict corresponding 
        // Feed Forward value for requested RPS values beyond max speed threshold
        double slope = (feedForwardVolts[lastElement] - feedForwardVolts[secondToLastElement]) / 
                        (feedForwardRPSThreshold[lastElement] - feedForwardRPSThreshold[secondToLastElement]);

        calculatedFeedForwardValue = slope * (desiredMotorRPS - feedForwardRPSThreshold[lastElement]) 
                                      + feedForwardVolts[lastElement];
      }
      // Linear interpolation to calculate a more precise Feed Forward value for 
      // points between established thresholds.
      // Calculated through weighted average.
      else {
        int upperBound = i + 1;
        int lowerBound = i;
        // Calculated weight based on distance between lower bound value and desired speed value
        double weight = (desiredMotorRPS - feedForwardRPSThreshold[upperBound]) /
                    (feedForwardRPSThreshold[lowerBound] - feedForwardRPSThreshold[upperBound]);
        
        calculatedFeedForwardValue = (weight * feedForwardVolts[lowerBound]) + 
                                      ((1 - weight) * feedForwardVolts[upperBound]);
      }

      // make sure wheel RPS shuffleboard inputs are in ascending order
      if (calculatedFeedForwardValue < 0) {
        calculatedFeedForwardValue = 0;
        kSVolts = 0;
      }

      // need to reverse Feed Forward value sign if speed is negative
      calculatedFeedForwardValue = calculatedFeedForwardValue * Math.signum(desiredMotorRPM);

      // convert speed to volts while accounting for volts required to overcome static friction
      double desiredVolts = (kSVolts * Math.signum(desiredMotorRPM)) + (calculatedFeedForwardValue * desiredMotorRPS);
      
      // send requested voltage to SparkMAX
      
      driveMotor.setVoltage(desiredVolts);
      /**if (error != REVLibError.kOk) {
        DriverStation.reportError("Drive motor " + driveMotor.getDeviceId() + " error " + error.name() + " while sending requested voltage", false);
      }**/
    }

    @Override
    public void setFeedForwardSpeedThreshold(double[] newFeedForwardRPSThreshold) {
      for (int i = 0; i < feedForwardRPSThreshold.length; i++) {
        feedForwardRPSThreshold[i] = newFeedForwardRPSThreshold[i];
      }
    }
    
    @Override
    public void updateFeedForward(double[] newFeedForwardVolts) {
      for (int i = 0; i < feedForwardVolts.length; i++) {
          feedForwardVolts[i] = newFeedForwardVolts[i];
      }
    }

    @Override
    public void updateVoltsToOvercomeFriction(double newkSVolts) {
      kSVolts = newkSVolts;
    }

    @Override
    public void setBrakeMode() {
      MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
      motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
      driveMotor.getConfigurator().refresh(motorOutputConfigs);
      turningMotor.getConfigurator().refresh(motorOutputConfigs);
    }

    @Override
    public void setCoastMode() {
      MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
      motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
      driveMotor.getConfigurator().refresh(motorOutputConfigs);
      turningMotor.getConfigurator().refresh(motorOutputConfigs);
    }

    @Override
    public void setClosedRampRate(double period) {
      ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
      closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = period;
      driveMotor.getConfigurator().refresh(closedLoopRampsConfigs);
    }

    @Override
    public void setOpenRampRate(double period) {
      OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
      openLoopRampsConfigs.VoltageOpenLoopRampPeriod = period;
      driveMotor.getConfigurator().refresh(openLoopRampsConfigs);
    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
    
}
