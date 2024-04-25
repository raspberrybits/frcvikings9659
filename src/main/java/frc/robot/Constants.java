package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class DriverConstants {
    public static final int leftRearId = 1;
    public static final int leftFrontId = 2;
    public static final int rightRearId = 4;
    public static final int rightFrontId = 3;
    public static final int currentLimit = 45;
    public static final int throughBoreCPR = 2048;
    public static final double trackWidth = 0.69;
    public static final double autoSpeed = 0.35;
    public static final double wheelDiameter = Units.inchesToMeters(6);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double gearRatio = 8.46;
    public static final double positionConversionFactor = wheelCircumference / gearRatio;
    public static final double velocityConversionFactor = wheelCircumference / (gearRatio*60);
    public static final double distancePerPulse = wheelCircumference / throughBoreCPR;
    public static final double kPL = 0.0001;
    public static final double kPR = 0.0001;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 2.30;
    public static final double kA = 0.49;
  }

  public static class OperatorConstants{
    public static final int driverPort = 1;
    public static final int controlsPort = 0;
  }

  public static class IntakeConstants{
    public static final int intakeFeedId = 7;
    public static final int rightIntakeId = 8;
    public static final int leftIntakeId = 9;
    public static final double speed = 1.0;
  }

  public static class ShooterConstants{
    public static final int shooterPrimeRightId  = 5;
    public static final int shooterPrimeLeftId  = 11;
    public static final int shooterTopFeedId = 6;
    public static final int shooterBottomFeedId = 7;
    public static final int ampHookId = 11;
    public static final double primeSpeed = 1.0 ;
    public static final double feedSpeed = 1.0;
    public static final double topIntakeSpeed = 0.5;
    public static final double delay = 5;
  }

  public static class HangConstants{
    public static final int hangId = 10;
    public static final double speed = 1;
    public static final double winchDistance = 450;
    public static final double unwinchDistance = 0;
  }
}
