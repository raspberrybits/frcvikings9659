// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class iDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain mDrivetrain;
  private DoubleSupplier left;
  private DoubleSupplier right;

  public iDrive(Drivetrain drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    mDrivetrain = drivetrain;
    left = leftSpeed;
    right = rightSpeed;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //SmartDashboard.putNumber("Left Joystick", left);
    //SmartDashboard.putNumber("Right Joystick", right);
    //SmartDashboard.putNumber("Target Yaw", mDrivetrain.getTargetYaw());
    mDrivetrain.drive(left.getAsDouble(), right.getAsDouble());
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
