package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.utility.OrangeMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {
  private SwerveModuleIO io;
  private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private WheelPosition wheelPos;

  private double previousRate = 0;
  private double previousTime = 0;
  private double filteredAccel = 0;

  public SwerveModule(WheelPosition wheelPos, SwerveModuleIO io) {
    this.io = io;
    this.wheelPos = wheelPos;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/SwerveModule " + wheelPos.wheelNumber, inputs);
    Logger.getInstance().recordOutput("Drive/SwerveModule " + wheelPos.wheelNumber + "/Drive1RotationsPerSecAbs", 
        Math.abs(inputs.driveRotationsPerSec));
  }

  public double getInternalRotationDegrees() {
    return OrangeMath.boundDegrees(inputs.turnDegrees);
  }

  public double getDistanceMeters() {
    return OrangeMath.falconRotationsToMeters(inputs.driveRotations,
        OrangeMath.getCircumference(OrangeMath.inchesToMeters(DriveConstants.Drive.wheelDiameterInches)),
        DriveConstants.Drive.gearRatio);
  }

  public double getVelocityFeetPerSec() {
    // feet per second
    return inputs.driveRotationsPerSec / Constants.DriveConstants.Drive.gearRatio 
        * Math.PI * Constants.DriveConstants.Drive.wheelDiameterInches / 12;
  }

  public double snapshotAcceleration() {

    double currentRate = this.getVelocityFeetPerSec();
    double currentTime = Timer.getFPGATimestamp();

    double acceleration = (currentRate - previousRate) / (currentTime - previousTime);

    previousRate = currentRate;
    previousTime = currentTime;

    filteredAccel = acceleration * 0.5 + filteredAccel * 0.5; // dampens random spikes due to the
                                                              // fact that we are deriving this
                                                              // value
    return filteredAccel;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityFeetPerSec() * Constants.feetToMeters, 
      Rotation2d.fromDegrees(inputs.turnDegrees));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistanceMeters(), Rotation2d.fromDegrees(inputs.turnDegrees));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Constants.driveEnabled) {

      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state =
          SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(inputs.turnDegrees));
      
      double desiredRPS = state.speedMetersPerSecond / (DriveConstants.Drive.wheelDiameterInches * Constants.inchesToMeters 
          * Math.PI) * DriveConstants.Drive.gearRatio;
          
      Logger.getInstance().recordOutput("Drive/SwerveModule " + wheelPos.wheelNumber + "/SetCalcMetersPerSec", 
          desiredState.speedMetersPerSecond);
      Logger.getInstance().recordOutput("Drive/SwerveModule " + wheelPos.wheelNumber + "/SetOptMetersPerSec", 
          state.speedMetersPerSecond);
      Logger.getInstance().recordOutput("Drive/SwerveModule " + wheelPos.wheelNumber + " /SetOptRPS", 
          desiredRPS);
      Logger.getInstance().recordOutput("Drive/SwerveModule " + wheelPos.wheelNumber + "/SetCalcDegrees", 
          desiredState.angle.getDegrees());
      Logger.getInstance().recordOutput("Drive/SwerveModule " + wheelPos.wheelNumber + "/SetOptDegrees", 
          state.angle.getDegrees());

      io.setDriveVoltage(desiredRPS * 60);
              
      if (!Constants.steeringTuningMode) {
        io.setTurnAngle(MathUtil.inputModulus(state.angle.getDegrees(), 0, 360));
      }
    }
  }

  public void setCoastmode() {
    if (Constants.driveEnabled) {
      io.setCoastMode();
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      io.setBrakeMode();
    }
  }

  public void setClosedRampRate(double rampRate) {
    io.setClosedRampRate(rampRate);
  }

  public void setOpenRampRate(double rampRate) {
    io.setOpenRampRate(rampRate);
  }

  public void stop() {
    if (Constants.driveEnabled) {
      if (!Constants.steeringTuningMode) {
        io.stopMotor();
      }
    }
  }

  public void updateFeedForward(double[] FFvalue) {
    io.updateFeedForward(FFvalue);
  }

  public void updateFFVelocityThreshold(double[] FFvelocityThreshold) {
    io.setFeedForwardSpeedThreshold(FFvelocityThreshold);
  }

  public void updateVoltsToOvercomeFriction(double kSVolts) {
    io.updateVoltsToOvercomeFriction(kSVolts);
  }
}
