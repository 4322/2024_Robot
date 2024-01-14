package frc.utility;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import frc.robot.Constants;

public class CanBusUtil {

  // stagger status frame periods to reduce peak CAN bus utilization
  private static int nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
  private static int nextShuffleboardStatusPeriodMs = Constants.shuffleboardStatusPeriodBaseMs;
  private static int nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
  private static int nextSparkVerySlowStatusPeriodMs = Constants.verySlowStatusPeriodSparkBaseMs;

  public static int nextFastStatusPeriodMs() {
    if (nextFastStatusPeriodMs > Constants.fastStatusPeriodMaxMs) {
      nextFastStatusPeriodMs = Constants.fastStatusPeriodBaseMs;
    }
    return nextFastStatusPeriodMs++;
  }

  public static int nextShuffleboardStatusPeriodMs() {
    if (nextShuffleboardStatusPeriodMs > Constants.shuffleboardStatusPeriodMaxMs) {
      nextShuffleboardStatusPeriodMs = Constants.shuffleboardStatusPeriodBaseMs;
    }
    return nextShuffleboardStatusPeriodMs++;
  }

  public static int nextSlowStatusPeriodMs() {
    if (nextSlowStatusPeriodMs > Constants.slowStatusPeriodMaxMs) {
      nextSlowStatusPeriodMs = Constants.slowStatusPeriodBaseMs;
    }
    return nextSlowStatusPeriodMs++;
  }

  // Don't use this for Talons because they can't go this slow
  public static int nextVerySlowStatusPeriodSparkMs() {
    nextSparkVerySlowStatusPeriodMs += 11;
    return nextSparkVerySlowStatusPeriodMs;
  }

  // Stagger status frames from SPARK MAX controllers.
  // Status frames needed at a higher rate can be set after initialization.
  public static void staggerSparkMax(CANSparkMax spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, nextSlowStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, nextVerySlowStatusPeriodSparkMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, nextVerySlowStatusPeriodSparkMs());
  }

  // configure for fast SparkMax encoder position feedback
  public static void fastPositionSparkMax(CANSparkMax spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, nextShuffleboardStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, nextShuffleboardStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, nextFastStatusPeriodMs());  // abs encoder pos
  }

  // configure for fast absolute encoder position feedback
  public static void fastPositionSparkMaxAbs(CANSparkMax spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, nextShuffleboardStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, nextShuffleboardStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, nextFastStatusPeriodMs());  // abs encoder pos
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, nextShuffleboardStatusPeriodMs());  // abs encoder vel
  }

  // configure for fast velocity feedback
  public static void fastVelocitySparkMax(CANSparkMax spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, nextShuffleboardStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, nextFastStatusPeriodMs());
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, nextShuffleboardStatusPeriodMs());
  }

  // Increase frame rates for a SPARK MAX that is being followed after initialization
  // when position control is in use.
  public static void dualSparkMaxPosCtrl(CANSparkMax mainMotor, boolean tuningMode) {
    // applied output for follower
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,
        nextFastStatusPeriodMs());
    // to detect when at target position
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2,
        nextFastStatusPeriodMs());
    if (tuningMode) {
      // to graph velocity
      mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
          nextFastStatusPeriodMs());
    }
  }

  // Increase frame rates for a SPARK MAX that is being followed after initialization
  // when velocity control is in use.
  public static void dualSparkMaxVelCtrl(CANSparkMax mainMotor, boolean tuningMode) {
    // applied output for follower
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0,
        nextFastStatusPeriodMs());
    // to detect when at target speed
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
        nextFastStatusPeriodMs());
    if (tuningMode) {
      // to graph position
      mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2,
          nextFastStatusPeriodMs());
    }
  }
}
