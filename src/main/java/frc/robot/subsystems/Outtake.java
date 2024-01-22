package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utility.OrangeMath;

public class Outtake extends SubsystemBase {
    private OuttakeIO io;
    private OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
    private Timer existenceTimer;
    private double pivotTarget;
    boolean initialized;

    public Outtake() {
        switch (Constants.currentMode) {
            case REAL:
                if (Constants.outtakeEnabled) {
                    io = new OuttakeIOReal();
                }
                break;
            case SIM:
                break;
            case REPLAY:
                break;
        }
        if (io == null) {
            io = new OuttakeIO() {
            };
        }

        existenceTimer = new Timer();
        existenceTimer.start();
    }

    public void periodic() {
         // initialize motor internal encoder position until the intake isn't moving
        if (Constants.outtakeEnabled && !initialized && !existenceTimer.hasElapsed(5)) {
            initialized = io.initPivot();
        }
        if (Constants.outtakeEnabled && initialized)
        {
            io.updateInputs(inputs);
            Logger.processInputs("Outtake", inputs);
            Logger.recordOutput("Outtake/TopRotationsPerSecAbs", Math.abs(inputs.topRotationsPerSec));
            Logger.recordOutput("Outtake/BottomRotationsPerSecAbs", Math.abs(inputs.bottomRotationsPerSec));
        }
    }
    public void pivot(double rotations)
    {
        if(Constants.outtakeEnabled && initialized)
        {
            io.setPivotTarget(rotations);
            pivotTarget = rotations;
            Logger.recordOutput("Outtake/PivotTargetRotations",rotations);
            Logger.recordOutput("Outtake/PivotStopped", false);

        }
    }
    public void resetPivot()
    {
        if (Constants.outtakeEnabled && initialized) {
            io.setPivotTarget(Constants.OuttakeConstants.defaultPivotPosition);
            pivotTarget = Constants.OuttakeConstants.defaultPivotPosition;
            Logger.recordOutput("Outtake/PivotTargetRotations",
                Constants.OuttakeConstants.defaultPivotPosition);
            Logger.recordOutput("Outtake/PivotStopped", false);
          }
    }
    public void outtake()
    {
        if(Constants.outtakeEnabled && initialized)
        {
            io.setOuttakePct(Constants.OuttakeConstants.TopOuttakePct, Constants.OuttakeConstants.BottomOuttakePct);
            Logger.recordOutput("Outtake/TopOuttakeTargetSpeedPct", Constants.OuttakeConstants.TopOuttakePct);
            Logger.recordOutput("Outtake/BottomOuttakeTargetSpeedPct", Constants.OuttakeConstants.BottomOuttakePct);
            Logger.recordOutput("Outtake/OuttakeStopped", false);
        }
    }
    public void stopOuttake()
    {
        if(Constants.outtakeEnabled && initialized)
        {
            io.stopOuttake();
            Logger.recordOutput("Outtake/TopOuttakeTargetSpeedPct", 0);
            Logger.recordOutput("Outtake/BottomOuttakeTargetSpeedPct", 0);
            Logger.recordOutput("Outtake/OuttakeStopped", true);
        }
    }
    public void stopPivot()
    {
        if(Constants.outtakeEnabled && initialized)
        {
            io.stopPivot();
            Logger.recordOutput("Outtake/PivotStopped", true);
        }
    }
    public void setCoastMode()
    {
        if(Constants.outtakeEnabled && initialized)
        {
            io.setCoastMode();
            Logger.recordOutput("Outtake/NeutralMode","Coast");
        }
    }
    public void setBrakeMode()
    {
    if(Constants.outtakeEnabled && initialized)
        {
            io.setBrakeMode();
            Logger.recordOutput("Outtake/NeutralMode","Brake");
        }
    }
    public boolean isAtPosition()
    {
          return OrangeMath.equalToEpsilon(inputs.pivotRotations, pivotTarget, Constants.OuttakeConstants.pivotToleranceRotations);

    }
}
