package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class ampShoot extends Command{
    Shooter mShooter;
    Intake mIntake;

    public ampShoot(Shooter shooter, Intake intake){
      mShooter = shooter;
      mIntake = intake;
      setName("pointAndShoot");
      addRequirements(shooter);
    }

    @Override
    public void initialize() {
      mShooter.setShooterPrime(0.06);
      mIntake.intakeFeed(ShooterConstants.primeSpeed);
      mShooter.setShooterFeed(ShooterConstants.primeSpeed);
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
