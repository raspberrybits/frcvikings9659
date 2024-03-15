package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkBase shooterPrimeRight = new CANSparkMax(ShooterConstants.shooterPrimeRightId, MotorType.kBrushless);
    private final CANSparkBase shooterPrimeLeft = new CANSparkMax(ShooterConstants.shooterPrimeLeftId, MotorType.kBrushless);
    private final VictorSPX shooterTopFeed = new VictorSPX(ShooterConstants.shooterTopFeedId);
    private final TalonSRX ampHook = new TalonSRX(ShooterConstants.ampHookId);

    public Shooter() {
    }

    public void setShooterFeed(double speed){
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void setShooterPrime(double speed){
        shooterPrimeRight.set(speed);
        shooterPrimeLeft.set(-speed);
    }

    public void topIntake(double speed){
        shooterPrimeRight.set(-speed*0.6);
        shooterPrimeLeft.set(speed*0.6);
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void extend(){}

    public void stop(){
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, 0);
        shooterPrimeRight.stopMotor();
        shooterPrimeLeft.stopMotor();
    }
}
