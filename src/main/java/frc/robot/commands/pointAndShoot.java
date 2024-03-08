package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class pointAndShoot extends Command{
    Shooter mShooter;
    Intake mIntake;

    public pointAndShoot(Shooter shooter, Intake intake){
        mShooter = shooter;
        mIntake = intake;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
      mShooter.setShooterPrime(ShooterConstants.primeSpeed);
      mIntake.intakeFeed(ShooterConstants.feedSpeed);
      mShooter.setShooterFeed(ShooterConstants.feedSpeed);
    }

    @Override
    public void execute() {}
  
    @Override
    public void end(boolean interrupted) {
      mShooter.stop();
      mIntake.intakeFeed(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
