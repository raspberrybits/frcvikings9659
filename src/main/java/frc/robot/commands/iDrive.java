// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class iDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain mDrivetrain;
  private double left;
  private double right;

  public iDrive(Drivetrain drivetrain, double leftSpeed, double rightSpeed) {
    mDrivetrain = drivetrain;
    left = leftSpeed;
    right = rightSpeed;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    mDrivetrain.drive(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
