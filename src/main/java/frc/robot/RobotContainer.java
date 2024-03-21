// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HangConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.pointAndShoot;
import frc.robot.commands.iDrive;
import frc.robot.commands.prime;
import frc.robot.commands.ampShoot;
import frc.robot.commands.floorIntake;
import frc.robot.commands.floorReverse;
import frc.robot.commands.hangRetract;
import frc.robot.commands.topIntake;
import frc.robot.commands.Autos.taxi;
import frc.robot.commands.Autos.shootPreload;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Shooter mShooter = new Shooter();
  private final Hang mHang = new Hang();
  
  private final SendableChooser<Command> autoChooser;

  //private final CommandXboxController mDriver = new CommandXboxController(OperatorConstants.driverPort);
  private final CommandXboxController mControls = new CommandXboxController(OperatorConstants.controlsPort);

  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("Shoot Preload", new shootPreload(mShooter, mIntake));
    autoChooser.addOption("Taxi", new taxi(mDrivetrain));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    //Tank Drive (single controller)
   mDrivetrain.setDefaultCommand(
        new iDrive(mDrivetrain, () -> -mControls.getLeftY(), () -> mControls.getRightY()));

    /* Tank Drive (2 controllers)  
    mDrivetrain.setDefaultCommand(

        new iDrive(mDrivetrain, () -> mDriver.getLeftY(), () -> -mDriver.getRightY()));*/

    // Amp Shoot
    mControls
        .a()
        .whileTrue(
            new ampShoot(mShooter, mIntake).withTimeout(0.5));
    
    // Shooter Prime
    mControls
        .leftTrigger()
        .whileTrue(
            new prime(mShooter));

    // Shooter Launch
    mControls
        .leftBumper()
        .whileTrue(
            new pointAndShoot(mShooter, mIntake));

    // Floor Intake
    mControls
        .rightTrigger()
        .whileTrue(
            new floorIntake(mIntake));

    // Floor Intake Reverse
    mControls
        .rightBumper()
        .whileTrue(
            new floorReverse(mIntake));

    // Top Intake
    mControls
        .x()
        .whileTrue(
            new topIntake(mShooter));

    //Hang Retract
    mControls
        .povDown()
        .whileTrue(
            new hangRetract(mHang, HangConstants.speed));

    //Hang Extend
    mControls
        .povUp()
        .whileTrue(
            new hangRetract(mHang, -HangConstants.speed));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
 