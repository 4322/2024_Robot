package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RobotCoordinator;
import frc.utility.OrangeMath;

public class Climber extends SubsystemBase{
    private static Climber climber;
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    public static Climber getInstance()
    {
        if(climber == null)
        {
            climber = new Climber();
        }
        return climber;
    }

    private Climber()
    {
        switch(Constants.currentMode)
        {
            case REAL:
                if(Constants.climberEnabled){
                    io = new ClimberIOReal();
                }
                break;
            case SIM:
                break;
            case REPLAY:
                break;
        }
        if(io == null)
        {
            io = new ClimberIO() {};
        }
    }
    @Override 
    public void periodic()
    {
        io.updateInputs(inputs);
    }
    public void extend()
    {
        if(Constants.climberEnabled)
        {
            setCoastMode();
            if(RobotCoordinator.getInstance().getSlowClimbButtonHeld()){
                io.setClimberVoltage(inputs.slowVolts);
                Logger.recordOutput("Climber/desiredVolts", inputs.slowVolts);
                Logger.recordOutput("Climber/State", "Extending");
            }
            else{
                io.setClimberVoltage(inputs.fastVolts);
                Logger.recordOutput("Climber/desiredVolts", inputs.fastVolts);
                Logger.recordOutput("Climber/State", "Extending");
            }
        }
    }
    public void retract()
    {
         if(Constants.climberEnabled)
        {
            setCoastMode();
            if(RobotCoordinator.getInstance().getSlowClimbButtonHeld()){
                io.setClimberVoltage(-inputs.slowVolts);
                Logger.recordOutput("Climber/desiredVolts", -inputs.slowVolts);
                Logger.recordOutput("Climber/State", "Retracting");
            }
            else{
                io.setClimberVoltage(-inputs.fastVolts);
                Logger.recordOutput("Climber/desiredVolts", -inputs.fastVolts);
                Logger.recordOutput("Climber/State", "Retracting");
            }
        }
    }
    public void setBrakeMode()
    {
        io.setBrakeMode();
        Logger.recordOutput("Climber/NeutralMode","Brake");
    }
    public void setCoastMode()
    {
        io.setCoastMode();
        Logger.recordOutput("Climber/NeutralMode","Coast");
    }
    public void stopClimb()
    {
        io.setBrakeMode();
        io.stopMotor();
        Logger.recordOutput("Climber/desiredVolts", 0);
        Logger.recordOutput("Climber/State","Stopped");
    }
    public boolean isFullyExtended()
    {
        return OrangeMath.equalToEpsilon(inputs.rotations, ClimberConstants.climberMaxRotations, ClimberConstants.climberRotationTolerance) || (inputs.rotations > ClimberConstants.climberMaxRotations);
    }
    public boolean isFullyRetracted()
    {
        return OrangeMath.equalToEpsilon(inputs.rotations, ClimberConstants.climberMinRotations, ClimberConstants.climberRotationTolerance) || (inputs.rotations < ClimberConstants.climberMinRotations);
    }
}
