// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class DriverConstants {
    public static final int leftRearId = 1;
    public static final int leftFrontId = 2;
    public static final int rightRearId = 4;
    public static final int rightFrontId = 3;
    public static final double trackWidth = 0.69;
  }

  public static class OperatorConstants{
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants{
    public static final int intakeFeedId = 7;
    public static final int rightIntakeId = 8;
    public static final int leftIntakeId = 9;
    public static final double speed = 1.0;
  }

  public static class ShooterConstants{
    public static final int shooterPrimeId  = 5;
    public static final int shooterTopFeedId = 6;
    public static final int shooterBottomFeedId = 7;
    public static final int ampHookId = 11;
    public static final double primeSpeed = 200.0;
    public static final double feedSpeed = 1.0;
    public static final double topIntakeSpeed = 0.5;
    public static final double delay = 1.0;

  }

  public static class HookConstants{
    public static final int hookId = 10;
  }
}
