package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangConstants;;

public class Hang extends SubsystemBase {
    private final CANSparkBase hookMotor = new CANSparkMax(HangConstants.hangId, MotorType.kBrushless);
    private final RelativeEncoder encoder = hookMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hang Encoder", encoder.getPosition());
    }

    public void drive(double speed) {
        hookMotor.set(speed);
    }

    public BooleanSupplier canWinch(){
        return () -> encoder.getPosition() < HangConstants.winchDistance;
    }

    public BooleanSupplier canUnwinch(){
        return () -> encoder.getPosition() > HangConstants.unwinchDistance;
    }

}
