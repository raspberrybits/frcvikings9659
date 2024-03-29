package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;


public class prime extends Command{
    Shooter mShooter;

    public prime(Shooter shooter){
      mShooter = shooter;
      setName("prime");
      addRequirements(shooter);
    }

    @Override
    public void initialize() {
      mShooter.setShooterPrime(ShooterConstants.primeSpeed);
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
