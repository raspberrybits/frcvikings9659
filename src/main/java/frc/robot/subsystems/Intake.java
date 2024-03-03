package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonSRX intakeLeft = new TalonSRX(IntakeConstants.leftIntakeId);
    private final TalonSRX intakeRight = new TalonSRX(IntakeConstants.rightIntakeId);  
    private final TalonSRX intakeFeed = new TalonSRX(IntakeConstants.intakeFeedId);
    public Intake() {
        intakeLeft.follow(intakeRight);
        intakeFeed.follow(intakeRight);
        intakeLeft.setInverted(true);
        intakeFeed.setInverted(true);
    }

    public void drive(double speed){
        intakeRight.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void intakeFeed(double speed){
        intakeFeed.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        intakeLeft.set(TalonSRXControlMode.PercentOutput, 0);
        intakeRight.set(TalonSRXControlMode.PercentOutput, 0);
        intakeFeed.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
