package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class floorReverse extends Command{
  Intake mIntake;
  
    public floorReverse(Intake intake){
        mIntake = intake;
        setName("floorReverse");
        addRequirements(intake);
    }

    @Override
    public void initialize() {
      mIntake.drive(-IntakeConstants.speed);
      mIntake.intakeFeed(-IntakeConstants.speed);
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
