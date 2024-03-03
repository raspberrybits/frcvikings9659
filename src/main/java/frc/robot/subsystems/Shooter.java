package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkBase shooterPrime = new CANSparkMax(ShooterConstants.shooterPrimeId, MotorType.kBrushless);
    private final TalonSRX shooterTopFeed = new TalonSRX(ShooterConstants.shooterTopFeedId);
    private final TalonSRX ampHook = new TalonSRX(ShooterConstants.ampHookId);

    public Shooter() {
        shooterPrime.setInverted(true);
        shooterTopFeed.setInverted(true);
    }

    public void setShooterFeed(double speed){
        shooterTopFeed.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void setShooterPrime(double speed){
        shooterPrime.set(speed);
    }

    public void topIntake(double speed){
        shooterPrime.set(speed);
        shooterTopFeed.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void extend(){}

    public void stop(){
        shooterTopFeed.set(TalonSRXControlMode.PercentOutput, 0);
        shooterPrime.stopMotor();
    }
}
