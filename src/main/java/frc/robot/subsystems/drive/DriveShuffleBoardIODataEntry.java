package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Constants.ControllerTypeStrings;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputScalingStrings;

public class DriveShuffleBoardIODataEntry implements DriveShuffleBoardIO {
  private ShuffleboardTab customizationTab;
  private GenericEntry closedRampRate;
  private GenericEntry openRampRate;
  private GenericEntry maxManualRotationEntry;
  private GenericEntry slowMovingAutoRotateEntry;
  private GenericEntry fastMovingAutoRotateEntry;
  private GenericEntry fastMovingFtPerSecEntry;
  private GenericEntry psuedoAutoRotateCheckbox;
  private ShuffleboardLayout voltsAtSpeedThresholdsLayout;
  private ShuffleboardLayout feedForwardMetersPerSecThresholdLayout;
  private GenericEntry voltsToOvercomeFrictionEntry;
  private SendableChooser<String> driveInputScaling;
  private SendableChooser<String> driveControlType;
  private GenericEntry[] feedForwardArray = new GenericEntry[DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds.length];
  private GenericEntry[] speedMetersPerSecArray = new GenericEntry[DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold.length];

  public DriveShuffleBoardIODataEntry() {

    if (Constants.debug) {
      // new shuffleboard tabs
      customizationTab = Shuffleboard.getTab("Drivebase Customization");
      
      // widgets for customizationTab
      psuedoAutoRotateCheckbox = customizationTab.add("Psuedo Auto Rotate", Constants.psuedoAutoRotateEnabled)
      .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 0).withSize(2, 1).getEntry();
      
      driveInputScaling = new SendableChooser<String>();
      driveInputScaling.addOption(InputScalingStrings.linear, InputScalingStrings.linear);
      driveInputScaling.setDefaultOption(InputScalingStrings.quadratic, InputScalingStrings.quadratic);
      driveInputScaling.addOption(InputScalingStrings.cubic, InputScalingStrings.cubic);

      customizationTab.add("Input Scaling", driveInputScaling).withWidget(BuiltInWidgets.kSplitButtonChooser)
          .withPosition(2, 0).withSize(3, 1);

      driveControlType = new SendableChooser<String>();
      driveControlType.addOption(ControllerTypeStrings.joysticks, ControllerTypeStrings.joysticks);
      driveControlType.setDefaultOption(ControllerTypeStrings.xboxLeftDrive, ControllerTypeStrings.xboxLeftDrive);
      driveControlType.addOption(ControllerTypeStrings.xboxRightDrive, ControllerTypeStrings.xboxRightDrive);

      customizationTab.add("Drive Control", driveControlType).withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(5, 0).withSize(3, 1);

      maxManualRotationEntry = customizationTab.add("Max Manual Rotate Power", 
          Constants.DriveConstants.Manual.maxManualRotation)
          .withPosition(0, 1).withSize(2, 1).getEntry();

      slowMovingAutoRotateEntry = customizationTab.add("Slow Moving Auto Rotate Power", 
          Constants.DriveConstants.Auto.slowMovingAutoRotate)
          .withPosition(2, 1).withSize(2, 1).getEntry();

      fastMovingAutoRotateEntry = customizationTab.add("Fast Moving Auto Rotate Power", 
          Constants.DriveConstants.Auto.fastMovingAutoRotate)
          .withPosition(4, 1).withSize(2, 1).getEntry();

      fastMovingFtPerSecEntry = customizationTab.add("Fast Moving Ft Per Sec", 
          Constants.DriveConstants.Auto.fastMovingFtPerSec)
          .withPosition(6, 1).withSize(2, 1).getEntry();
          
      closedRampRate = customizationTab.add("Acc Ramp Rate", DriveConstants.Drive.closedLoopRampSec)
          .withPosition(0, 2).withSize(1, 1).getEntry();

      openRampRate = customizationTab.add("Stop Ramp Rate", DriveConstants.Drive.openLoopRampSec)
          .withPosition(1, 2).withSize(1, 1).getEntry();
      
      voltsToOvercomeFrictionEntry = customizationTab.add("Volts Required to Overcome Friction", 
          DriveConstants.Drive.kS).withPosition(6,2).withSize(2,2).getEntry();

      // Enables manipulation of arrays in Shuffleboard GUI and displays nicely
      voltsAtSpeedThresholdsLayout = customizationTab.getLayout("Volts over m per s at Speed Thresholds", BuiltInLayouts.kList)
          .withPosition(8,1).withSize(2,3);
      for (int i = 0; i < DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds.length; i++) {
        feedForwardArray[i] = voltsAtSpeedThresholdsLayout.add("Volts over m per s " + i, DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds[i]).getEntry();
      }

      feedForwardMetersPerSecThresholdLayout = customizationTab.getLayout("FF Threshold m per s", BuiltInLayouts.kList)
          .withPosition(10,1).withSize(2,3);
      for (int i = 0; i < DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold.length; i++) {
        speedMetersPerSecArray[i] = feedForwardMetersPerSecThresholdLayout.add("Speed m per s " + i, DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold[i]).getEntry();
      }
    }
  }
    

  @Override
  public void updateInputs(DriveShuffleBoardIOInputs inputs) {
    if (Constants.debug) {
      inputs.psuedoAutoRotateEnabled = psuedoAutoRotateCheckbox.getBoolean(Constants.psuedoAutoRotateEnabled);
      inputs.inputScaling = driveInputScaling.getSelected();
      inputs.driveControllerType = driveControlType.getSelected();
      inputs.maxManualRotatePower = maxManualRotationEntry.getDouble(Constants.DriveConstants.Manual.maxManualRotation);
      inputs.slowMovingAutoRotatePower = slowMovingAutoRotateEntry.getDouble(Constants.DriveConstants.Auto.slowMovingAutoRotate);
      inputs.fastMovingAutoRotatePower = fastMovingAutoRotateEntry.getDouble(Constants.DriveConstants.Auto.fastMovingAutoRotate);
      inputs.fastMovingFtPerSec = fastMovingFtPerSecEntry.getDouble(Constants.DriveConstants.Auto.fastMovingFtPerSec);
      inputs.accelerationRampRate = closedRampRate.getDouble(DriveConstants.Drive.closedLoopRampSec);
      inputs.stoppedRampRate = openRampRate.getDouble(DriveConstants.Drive.openLoopRampSec);
      for (int i = 0; i < DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds.length; i++) {
        inputs.voltsOverMetersPerSecAtSpeedThresholds[i] = feedForwardArray[i].getDouble(DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds[i]);
      }
      for (int i = 0; i < DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold.length; i++) {
        inputs.feedForwardMetersPerSecThresholds[i] = speedMetersPerSecArray[i].getDouble(DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold[i]);
      }
      inputs.voltsToOvercomeFriction = voltsToOvercomeFrictionEntry.getDouble(DriveConstants.Drive.kS);
    }
    else { 
      // if debug not enabled, don't want values to be 0
      inputs.psuedoAutoRotateEnabled = Constants.psuedoAutoRotateEnabled;
      inputs.inputScaling = Constants.driveInputScaling;
      inputs.driveControllerType = Constants.controllerType;
      inputs.maxManualRotatePower = Constants.DriveConstants.Manual.maxManualRotation;
      inputs.slowMovingAutoRotatePower = Constants.DriveConstants.Auto.slowMovingAutoRotate;
      inputs.fastMovingAutoRotatePower = Constants.DriveConstants.Auto.fastMovingAutoRotate;
      inputs.fastMovingFtPerSec = Constants.DriveConstants.Auto.fastMovingFtPerSec;
      inputs.accelerationRampRate = DriveConstants.Drive.closedLoopRampSec;
      inputs.stoppedRampRate = DriveConstants.Drive.openLoopRampSec;
      inputs.voltsOverMetersPerSecAtSpeedThresholds = DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds;
      inputs.feedForwardMetersPerSecThresholds = DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold;
      inputs.voltsToOvercomeFriction = DriveConstants.Drive.kS;
    }
  }
}
