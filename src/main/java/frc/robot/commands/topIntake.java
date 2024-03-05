package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class topIntake extends Command{
    Shooter mShooter;

    public topIntake(Shooter shooter){
        mShooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        mShooter.setShooterPrime(-ShooterConstants.topIntakeSpeed);
        mShooter.setShooterFeed(-ShooterConstants.topIntakeSpeed);
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
