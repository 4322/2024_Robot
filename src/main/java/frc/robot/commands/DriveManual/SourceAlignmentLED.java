package frc.robot.commands.DriveManual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveManual.DriveManualStateMachine.DriveManualState;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.limelight.Limelight;

public class SourceAlignmentLED extends Command {
    private DriveManual driveManual;

    public SourceAlignmentLED(DriveManual driveManual) {
        this.driveManual = driveManual;
        addRequirements(LED.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        final int leftTagID;
        final int rightTagID;
        if (Robot.isRed()) {
            leftTagID = Constants.FieldConstants.redLeftSourceTagID;
            rightTagID = Constants.FieldConstants.redRightSourceTagID;
        }
        else {
            leftTagID = Constants.FieldConstants.blueLeftSourceTagID;
            rightTagID = Constants.FieldConstants.blueRightSourceTagID;
        }

        if (Limelight.getOuttakeInstance().getSpecifiedTagVisible(leftTagID) 
            || Limelight.getOuttakeInstance().getSpecifiedTagVisible(rightTagID)) {
            if (Math.abs(Limelight.getOuttakeInstance().getTag(leftTagID).tx) <= 5.0 
                || Math.abs(Limelight.getOuttakeInstance().getTag(rightTagID).tx) <= 5.0) {
                LED.getInstance().setLEDState(LED.LEDState.alignedWithAmp);
            }
            else {
                LED.getInstance().setLEDState(LED.LEDState.aligningWithAmp);
            }
        }
        else {
            LED.getInstance().setLEDState(LED.LEDState.idle);
        }
    }

    @Override
    public boolean isFinished() {
        return driveManual.getState() != DriveManualState.AMP;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
