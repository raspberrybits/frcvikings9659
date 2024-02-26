package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
    
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkBase intakeLeft = new CANSparkMax(IntakeConstants.leftIntakeId, MotorType.kBrushed);
    private final CANSparkBase intakeRight = new CANSparkMax(IntakeConstants.rightIntakeId, MotorType.kBrushed);  
    private final CANSparkBase intakeFeed = new CANSparkMax(IntakeConstants.intakeFeedId, MotorType.kBrushed);

    public Intake() {
        intakeLeft.follow(intakeRight);
        intakeFeed.follow(intakeRight);
        intakeLeft.setInverted(true);
        intakeFeed.setInverted(true);
    }

    public void drive(double speed){
        speed = speed > 1 || speed < -1 ? 1 * (speed/Math.abs(speed)) : speed;
        intakeRight.set(speed);
    }

    public void stop() {
        intakeLeft.stopMotor();
        intakeRight.stopMotor();
        intakeFeed.stopMotor();
    }
}
