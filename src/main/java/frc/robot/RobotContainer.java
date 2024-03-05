// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.pointAndShoot;
import frc.robot.commands.iDrive;
import frc.robot.commands.prime;
import frc.robot.commands.floorIntake;
import frc.robot.commands.topIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Shooter mShooter = new Shooter();

  private final CommandXboxController mController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // Tank Drive
    mDrivetrain.setDefaultCommand(
        new iDrive(mDrivetrain, () -> mController.getLeftY(), () -> -mController.getRightY()));

    /*
     //Amp Shoot
     mController
     .x()
     .whileTrue(
     new prime(mShooter)
     .withTimeout(ShooterConstants.delay)
     .andThen(new pointAndShoot(mShooter))
     .handleInterrupt(() -> mShooter.stop()));
    */

    // Shooter Prime
    mController
        .rightTrigger()
        .whileTrue(
            new prime(mShooter)
                .handleInterrupt(() -> mShooter.stop()));

    // Shooter Launch
    mController
        .rightBumper()
        .whileTrue(
            new pointAndShoot(mShooter, mIntake)
                .handleInterrupt(() -> mShooter.stop()));

    // Floor Intake
    mController
        .leftBumper()
        .whileTrue(
            new floorIntake(mIntake)
                .handleInterrupt(() -> mIntake.stop()));

    // Top Intake
    mController
        .b()
        .whileTrue(
            new topIntake(mShooter)
                .handleInterrupt(() -> mShooter.stop()));

    // Amp Shoot
    mController
        .a()
        .onTrue(
            new prime(mShooter)
                .withTimeout(ShooterConstants.delay)
                .andThen()
                .withTimeout(0)
                .andThen()
                .handleInterrupt(() -> mShooter.stop()));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Auto 1");
  }

}
