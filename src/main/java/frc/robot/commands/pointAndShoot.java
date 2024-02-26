package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class pointAndShoot extends Command{
    Shooter mShooter;

    public pointAndShoot(Shooter shooter){
        mShooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
      mShooter.setShooterPrime(ShooterConstants.primeSpeed);
      mShooter.setShooterFeed(ShooterConstants.feedSpeed);
    }

    @Override
    public void execute() {
      
    }
  
    @Override
    public void end(boolean interrupted) {
      mShooter.stop();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
