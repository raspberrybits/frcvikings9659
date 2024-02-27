package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkBase shooterPrime = new CANSparkMax(ShooterConstants.shooterPrimeId, MotorType.kBrushless);
    private final CANSparkBase shooterTopFeed = new CANSparkMax(ShooterConstants.shooterTopFeedId, MotorType.kBrushed);
    private final CANSparkBase ampHook = new CANSparkMax(ShooterConstants.ampHookId, MotorType.kBrushless);

    public Shooter() {
        shooterPrime.setInverted(true);
        shooterTopFeed.setInverted(true);
    }

    public void setShooterFeed(double speed){
        shooterTopFeed.set(speed);
    }

    public void setShooterPrime(double speed){
        shooterPrime.set(speed);
    }

    public void topIntake(double speed){
        shooterPrime.set(speed);
        shooterTopFeed.set(speed);
    }

    public void extend(){}

    public void stop(){
        shooterTopFeed.stopMotor();
        shooterPrime.stopMotor();
    }
}
