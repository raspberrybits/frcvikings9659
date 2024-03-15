package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class iDrive extends Command {
  private Drivetrain mDrivetrain;
  private DoubleSupplier left;
  private DoubleSupplier right;

  public iDrive(Drivetrain drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    mDrivetrain = drivetrain;
    left = leftSpeed;
    right = rightSpeed;
    setName("iDrive");
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
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
