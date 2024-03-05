package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final VictorSPX intakeLeft = new VictorSPX(IntakeConstants.leftIntakeId);
    private final TalonSRX intakeRight = new TalonSRX(IntakeConstants.rightIntakeId);  
    private final VictorSPX intakeFeed = new VictorSPX(IntakeConstants.intakeFeedId);
    public Intake() {
        intakeLeft.follow(intakeRight);
        intakeFeed.follow(intakeRight);
        intakeLeft.setInverted(true);
        intakeFeed.setInverted(true);
    }

    public void drive(double speed){
        intakeRight.set(TalonSRXControlMode.PercentOutput, -speed);
        intakeLeft.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void intakeFeed(double speed){
        intakeFeed.set(VictorSPXControlMode.PercentOutput, -speed);
    }

    public void stop() {
        intakeLeft.set(VictorSPXControlMode.PercentOutput, 0);
        intakeRight.set(TalonSRXControlMode.PercentOutput, 0);
        intakeFeed.set(VictorSPXControlMode.PercentOutput, 0);
    }
}
