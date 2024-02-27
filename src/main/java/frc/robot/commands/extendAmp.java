package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class extendAmp extends Command{
    Shooter mShooter;

    public extendAmp(Shooter shooter){
        mShooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        mShooter.extend();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
