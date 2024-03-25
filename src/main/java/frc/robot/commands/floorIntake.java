package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class floorIntake extends Command{
  Intake mIntake;
  
    public floorIntake(Intake intake){
        mIntake = intake;
        setName("floorIntake");
        addRequirements(intake);
    }

    @Override
    public void initialize() {
      mIntake.drive(IntakeConstants.speed);
      mIntake.intakeFeed(IntakeConstants.speed);
    }
    
    @Override
    public void end(boolean interrupted) {
      mIntake.stop();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
