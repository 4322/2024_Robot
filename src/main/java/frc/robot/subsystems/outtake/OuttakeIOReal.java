package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class OuttakeIOReal implements OuttakeIO {
  private TalonFX topOuttakeMotor;
  private TalonFX bottomOuttakeMotor;
  // shuffleboard
  ShuffleboardTab tab;
  GenericEntry outtakeFlywheelSpeed;

  public OuttakeIOReal() {
    topOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.leftOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    bottomOuttakeMotor =
        new TalonFX(
            Constants.OuttakeConstants.rightOuttakeDeviceID,
            Constants.DriveConstants.Drive.canivoreName);
    configOuttake(topOuttakeMotor);
    configOuttake(bottomOuttakeMotor);
    if (Constants.debug) {
      tab = Shuffleboard.getTab("Outtake");
      outtakeFlywheelSpeed =
          tab.add("Desired Flywheel Velocity (RPS)", 0)
              .withSize(1, 1)
              .withPosition(0, 0)
              .getEntry();
    }
  }

  private void configOuttake(TalonFX talon) {
    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.OuttakeConstants.kP;
    slot0Configs.kI = Constants.OuttakeConstants.kI;
    slot0Configs.kD = Constants.OuttakeConstants.kD;
    slot0Configs.kV = Constants.OuttakeConstants.kF;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        Constants.OuttakeConstants.closedLoopRampSec;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = Constants.OuttakeConstants.openLoopRampSec;
    // bottomOuttakeMotor.setControl(new
    // Follower(topOuttakeMotor.getDeviceID(),true));
    talon.getConfigurator().apply(slot0Configs);
    talon.getConfigurator().apply(closedLoopRampsConfigs);
    talon.getConfigurator().apply(openLoopRampsConfigs);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.topCurrentAmps = topOuttakeMotor.getSupplyCurrent().getValue();
    inputs.topTempC = topOuttakeMotor.getDeviceTemp().getValue();
    inputs.topRotationsPerSec = topOuttakeMotor.getVelocity().getValue();

    inputs.bottomCurrentAmps = bottomOuttakeMotor.getSupplyCurrent().getValue();
    inputs.bottomTempC = bottomOuttakeMotor.getDeviceTemp().getValue();
    inputs.bottomRotationsPerSec = bottomOuttakeMotor.getVelocity().getValue();
    inputs.bottomOuttakeIsAlive = bottomOuttakeMotor.isAlive();
    inputs.topOuttakeIsAlive = topOuttakeMotor.isAlive();
    if (Constants.debug) inputs.debugTargetRPS = outtakeFlywheelSpeed.getDouble(0);
  }

  @Override
  public void setOuttakeRPS(double desiredTopVelocityRPS, double desiredBottomVelocityRPS) {
    topOuttakeMotor.setControl(new VelocityVoltage(desiredTopVelocityRPS));
    bottomOuttakeMotor.setControl(new VelocityVoltage(desiredBottomVelocityRPS));
  }

  @Override
  public void setBrakeMode() {
    MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
    mOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    topOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    bottomOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    Logger.recordOutput("Outtake/Hardware/NeutralMode", "Brake");
  }

  @Override
  public void setCoastMode() {
    MotorOutputConfigs mOutputConfigs = new MotorOutputConfigs();
    mOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    topOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    bottomOuttakeMotor.getConfigurator().refresh(mOutputConfigs);
    Logger.recordOutput("Outtake/Hardware/NeutralMode", "Coast");
  }

  @Override
  public void stopOuttake() {
    topOuttakeMotor.stopMotor();
    bottomOuttakeMotor.stopMotor();
  }
}
